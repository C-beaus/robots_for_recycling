#!/usr/bin/env python3

# from PandaManipulator import PandaManipulator
import rospy
from std_msgs.msg import Float64, Float64MultiArray
import argparse
from robots_for_recycling.srv import ClassifySrv, GraspSrv, CameraSrv, rgbdSrv, SuctionSrv, rgbdSrvRequest, GraspSrvRequest, ClassifySrvRequest, SuctionSrvRequest, PandaSrv, PandaSrvRequest
import pyrealsense2 as rs
import numpy as np
import cv2
from concurrent.futures import ThreadPoolExecutor
import threading
from cv_bridge import CvBridge



class TaskPlanner:
    def __init__(self, calibrate_bool):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        self.drift_speed = 0
        self.executor = ThreadPoolExecutor()

        self.franka_running = False  # Reentrancy guard for franka
        self.cartesian_running = False 

        if calibrate_bool:
            self.calibrate_franka = rospy.Subscriber("/calibrate", Float64, self.calibrate_franka)
            self.bridge = CvBridge()
            print("Ready to calibrate")

            
        else:
            self.conveyor_speed_sub = rospy.Subscriber('/conveyor_speed', Float64, self.conveyor_speed_callback)

            self.franka_timer = rospy.Timer(rospy.Duration(5), self.run_franka)
            rospy.sleep(0.5)
            self.cartesian_timer = rospy.Timer(rospy.Duration(5), self.run_cartesian)

            rospy.on_shutdown(self.shutdown)
        
        # self.pandaManipulator = PandaManipulator()
        # self.pandaEndEffector = self.pandaManipulator.end_effector
        

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
            if response:
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
            rospy.loginfo(f"response info: {response}")

            # Check and handle the response
            if response:
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
            request.bbs.data = bboxes
            
            # Call the service
            rospy.loginfo("Calling the antipodal grasp generation service to select grasps within given bounding boxes...")
            response = get_grasps(request) # Flat Grasps need to be reshaped using response.rehsape(-1, 6) by manipualtor node

            # Check and handle the response
            if response:
                rospy.loginfo("Grasp Selection completed successfully. Ready to execute grasps.")
                return response.grasps.data
            else:
                rospy.logwarn("Grasp Selection did not succeed.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to anipodal grasp selection service failed: {e}")

    def call_franka_robot_service(self, grasps):
        # print(grasps)
        rospy.wait_for_service('panda_control')
        try:
            panda_control = rospy.ServiceProxy('panda_control', PandaSrv)
            request = PandaSrvRequest()
            request.grasps.data = grasps
            response = panda_control(request)
        except:
            rospy.loginfo("panda control service failed")
    
    def call_cartesian_robot_service(self, grasps):
        raise NotImplementedError
    
    def call_suction_grasp_service(self, depth_image, bboxes):
        
        # Wait for the service to become available
        rospy.wait_for_service('get_sucked')
        try:

            get_suction_grasps = rospy.ServiceProxy('get_sucked', SuctionSrv)
            request = SuctionSrvRequest()
            request.bbs.data = bboxes
            request.depth_image = depth_image
            
            # Call the service
            rospy.loginfo("Calling the suction grasp generation service to generate grasps within given bounding boxes...")
            response = get_suction_grasps(request) # Flat Grasps need to be reshaped using response.rehsape(-1, 4) by manipualtor node

            # Check and handle the response
            if response:
                rospy.loginfo("Suction Grasp generation completed successfully. Ready to execute grasps.")
                return response.grasps.data
            else:
                rospy.logwarn("Grasp generation did not succeed.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to suction grasp selection service failed: {e}")
    
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
            if response:
                rospy.loginfo("Classification completed successfully. Ready to call grasp selection service using bboxes.")
                return response.bbs.data
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


    def calibrate_franka(self, msg):

        counter = 0
        while True:
            # Capture camera frames
            # Capture camera frames
            if counter <10:
                rgb_image, depth_image, timestamp = self.call_camera_service()

                if rgb_image and depth_image:
                    # Run classification and grasp inference in parallel
                    tasks = [
                        (self.call_classification_service, rgb_image),
                        (self.call_grasp_inference_service, depth_image, rgb_image)
                    ]
                    results = self.run_parallel_tasks(tasks, self.executor)

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


                radius = 50          # Radius of the circle
                color = (0, 0, 255)   # Red color in BGR (OpenCV uses BGR, not RGB)
                thickness = 2         # Thickness of the circle's edge (use -1 for a filled circle)

                grasps = np.asarray(grasps)
                rgb_frame = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding="rgb8")


                # Draw the circle on the image

                # print(f"here are the boxes: {grasps}")
                # print(f"type is: {type(grasps)}")
                # print(f"dir is: {dir(grasps)}")
                # print(f"output: {yoloV5_data.output}")
                # print(f"output.data {grasps.output.data}")

                ppx=321.1669921875
                ppy=231.57203674316406
                fx=605.622314453125
                fy=605.8401489257812

                center_z = grasps[2]
                center_x = (grasps[0]/center_z) * fx + ppx
                center_y = (grasps[1]/center_z) * fy + ppy
                print(f"x: {center_x}, y: {center_y}, z: {center_z}")

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(rgb_frame, f'x: {center_x}, y: {center_y}, z: {center_z}', (50, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                center = (int(center_x),int(center_y))
                print(f"center is: {center}")
                # print(rgb_image)
                cv2.circle(rgb_frame, center, radius, color, thickness)
                cv2.circle(rgb_frame, center, 2, color, 1)

                print("going to show")

                # Display the image with the circle
                cv2.imshow('Live Image', rgb_frame)

                # Press 'q' to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            counter+=1


    def run_franka(self, event):

        if self.franka_running:
            rospy.loginfo("run_franka callback is already running. Skipping this cycle.")
            return
        
        self.franka_running = True
        
        try:
            # Capture camera frames
            rgb_image, depth_image, timestamp = self.call_camera_service()

            if rgb_image and depth_image:
                # Run classification and grasp inference in parallel
                tasks = [
                    (self.call_classification_service, rgb_image),
                    (self.call_grasp_inference_service, depth_image, rgb_image)
                ]
                results = self.run_parallel_tasks(tasks, self.executor)

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
        except Exception as e:
            rospy.logerr(f"Error running run_franka callback: {e}")
        finally:
            self.franka_running = False

    
    def run_cartesian(self, event):

        if self.cartesian_running:
            rospy.loginfo("run_cartesian callback is already running. Skipping this cycle.")
            return

        self.cartesian_running = True

        try:

            # Capture camera frames
            rgb_image, depth_image, timestamp = self.call_camera_service()

            if rgb_image and depth_image:

                # Run classification first
                bboxes = self.call_classification_service(rgb_image)

                # Handle classification results
                if not bboxes:
                    rospy.logwarn("No bounding boxes detected from classification. Exiting run_cartesian function.")
                    return

                print("here")
                suction_grasps = self.call_suction_grasp_service(depth_image, bboxes) # This is a flat array. needs to be reshaped like 
                                                                                    # grasps.reshape(-1, 3) where each  row would then become [x, y, z]

                print("suction grasps collected")
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

        except Exception as e:
            rospy.logerr(f"Error running run_cartesian callback: {e}")
        finally:
            self.cartesian_running = False

    def shutdown(self):
        rospy.loginfo("Shutting down task planner")
        rospy.loginfo("Shutting down executor")
        self.executor.shutdown(wait=True)


    def run(self):
        try:
            # spinner = rospy.AsyncSpinner(2) # pass in number of threads
            # spinner.start()
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down task planner")
            rospy.loginfo("Shutting down executor")

if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument(
         "-m",
         "--manipulator_type",
         default=None,
         help="What manipulator do you want to use for recycling?"
    )

    parser.add_argument(
        "-e",
        "--end_effector",
        default=None,
        help="What end effector is on the manipulator you are using?"
    )

    parser.add_argument(
        "-c",
        "--calibrate_bool",
        default=False,
        help="Set to True to manually calibrate the FRANKA arm"
    )

    args = parser.parse_args()
    # args.calibrate_bool = True

    TaskPlanner(args.calibrate_bool).run()
