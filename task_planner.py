#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import argparse
from robots_for_recycling.srv import ClassifySrv, GraspSrv, CameraSrv, rgbdSrv, rgbdSrvRequest, GraspSrvRequest, ClassifySrvRequest
import pyrealsense2 as rs
import numpy as np
import cv2
from concurrent.futures import ThreadPoolExecutor



class TaskPlanner:
    def __init__(self):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        self.drift_speed = 0
        self.executor_franka = ThreadPoolExecutor()
        self.conveyor_speed_sub = rospy.Subscriber('float32_topic', Float64, self.conveyor_speed_callback)
        franka_timer = rospy.Timer(rospy.Duration(20), self.run_franka)
        cartesian_timer = rospy.Timer(rospy.Duration(20), self.run_cartesian)

    def conveyor_speed_callback(self, msg):
        self.drif_speed = msg.data

    def call_camera_service(self):
        
        # Wait for the service to become available
        rospy.wait_for_service('camera_service')
        try:

            get_rgbd_frames = rospy.ServiceProxy('camera_service', CameraSrv)

            # Call the service
            rospy.loginfo("Calling the camera service to get RGB and depth frames...")
            response = get_rgbd_frames()

            # Check and handle the response
            if response.success:
                rospy.loginfo("RGB and Depth pair received. Ready For Classification and Grasp Generation")
                return response.rgb_image, response.depth_image, response.timestamp
            else:
                rospy.logwarn("Capture of RGB and Depth frames was unsuccessful.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to the camera service failed: {e}")
    
    def call_grasp_inference_service(self, depth_image, rgb_image):

        # Wait for the service to become available
        rospy.wait_for_service('run_grasp_model')
        try:

            run_antipodal_network = rospy.ServiceProxy('run_grasp_model', rgbdSrv)
            request = rgbdSrvRequest()
            request.rgb_image = rgb_image
            request.depth_image = depth_image
            
            # Call the service
            rospy.loginfo("Calling the antipodal model service to run inference on current frames...")
            response = run_antipodal_network(request)

            # Check and handle the response
            if response.success:
                rospy.loginfo("Antipodal inference completed successfully. Ready to receive bounding boxes.")
                return response.infer_success
            else:
                rospy.logwarn("Antipodal inference did not succeed.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to anipodal inference service failed: {e}")

    def call_grasp_selection_service(self, bboxes):

        # Wait for the service to become available
        rospy.wait_for_service('select_grasps_from_bbs')
        try:

            get_grasps = rospy.ServiceProxy('select_grasps_from_bbs', GraspSrv)
            request = GraspSrvRequest()
            request.bbs = bboxes
            
            # Call the service
            rospy.loginfo("Calling the antipodal grasp generation service to select grasps within given bounding boxes...")
            response = get_grasps(request) # Flat Grasps need to be reshaped using response.rehsape(-1, 6) by manipualtor node

            # Check and handle the response
            if response.success:
                rospy.loginfo("Grasp Selection completed successfully. Ready to execute grasps.")
                return response.grasps
            else:
                rospy.logwarn("Grasp Selection did not succeed.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to anipodal grasp selection service failed: {e}")

    def call_franka_robot_service(self, grasps):
        raise NotImplementedError
    
    def call_cartesian_robot_service(self, grasps):
        raise NotImplementedError
    
    def call_suction_grasp_service(self, rgb_image, bboxes):
        raise NotImplementedError
    
    def call_classification_service(self, rgb_image):

        # Wait for the service to become available
        rospy.wait_for_service('classify_waste')
        try:

            get_bboxes = rospy.ServiceProxy('classify_waste', ClassifySrv)
            request = ClassifySrvRequest()
            request.rgb_image = rgb_image
            
            # Call the service
            rospy.loginfo("Calling the classification service to generate bounding boxes...")
            response = get_bboxes(request)

            # Check and handle the response
            if response.success:
                rospy.loginfo("Classification completed successfully. Ready to call grasp selection service using bboxes.")
                return response.bbs
            else:
                rospy.logwarn("Classification did not succeed.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to classification service failed: {e}")
    
    def run_parallel_tasks(self, tasks, executor):
        futures = [self.executor.submit(task[0], *task[1:]) for task in tasks]
        results = []
        for future in futures:
            try:
                results.append(future.result())
            except Exception as e:
                rospy.logerr(f"Error in parallel task execution: {e}")
        return results



    def run_franka(self, event):
        
        # Capture camera frames
        rgb_image, depth_image, timestamp = self.call_camera_service()

        if rgb_image and depth_image:
            # Run classification and grasp inference in parallel
            tasks = [
                (self.call_classification_service, rgb_image),
                (self.call_grasp_inference_service, depth_image, rgb_image)
            ]
            results = self.run_parallel_tasks(tasks, self.executor_franka)

            # Handle classification results
            bboxes = results[0] if results[0] else None
            if not bboxes:
                rospy.logwarn("No bounding boxes detected from classification. Exiting run_franka function.")
                return

            # Handle grasp inference results
            if not results[1]:
                rospy.logwarn("Grasp network's inference failed. Exiting run_franka function.")
                return

            rospy.loginfo(f"Results from antipodal grasp service revcevied.")

        else:
            rospy.logwarn("RGB and Depth Frames did not arrive. Check Service.")
            return

        # Perform grasp selection
        grasps = self.call_grasp_selection_service(bboxes) # This is a flat array. needs to be reshaped like grasps.reshape(-1, 6) where each
                                                           # row would then become [x, y, z, angle, witdh, label]

        if grasps:
            rospy.loginfo("Grasps received for given objects.")
        else:
            rospy.logwarn("No grasps received. Exiting run_franka function.")
            return

        # Try to execute grasps. All grasp offsets must be handled within the franka/cartesian service.
        # The current grasps are computed at the object, so they need a little offset to not collide with the object.
        self.call_franka_robot_service(grasps)

    
    def run_cartesian(self, event):

        # Capture camera frames
        rgb_image, depth_image, timestamp = self.call_camera_service()

        if rgb_image and depth_image:

            # Run classification first
            bboxes = self.call_classification_service(rgb_image)

            # Handle classification results
            if not bboxes:
                rospy.logwarn("No bounding boxes detected from classification. Exiting run_cartesian function.")
                return

            suction_grasps = self.call_suction_grasp_service(rgb_image, bboxes)

            # Handle suciton grasp results
            if not suction_grasps:
                rospy.logwarn("Suction grasps not found. Exiting run_cartesian function.")
                return

            rospy.loginfo(f"Grasps received for given objects.")

        else:
            rospy.logwarn("RGB and Depth Frames did not arrive. Check Service.")
            return

        # Try to execute grasps. All grasp offsets must be handled within the franka/cartesian service.
        # The current grasps are computed at the object, so they need a little offset to not collide with the object.
        self.call_cartesian_robot_service(suction_grasps)


    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down executor")
            self.executor_franka.shutdown(wait=True)

if __name__ == '__main__':
    TaskPlanner().run()