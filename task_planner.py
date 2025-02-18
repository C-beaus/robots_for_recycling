#!/usr/bin/env python3

# from PandaManipulator import PandaManipulator
import rospy
from std_msgs.msg import Float64, Float64MultiArray
import argparse
from robots_for_recycling.srv import ClassifySrv, GraspSrv, CameraSrv, rgbdSrv, SuctionSrv, rgbdSrvRequest, GraspSrvRequest, ClassifySrvRequest, SuctionSrvRequest, PandaSrv, PandaSrvRequest, GantrySrv, GantrySrvRequest
import pyrealsense2 as rs
import numpy as np
import cv2
from concurrent.futures import ThreadPoolExecutor
import threading
from cv_bridge import CvBridge
from time import sleep


class TaskPlanner:
    def __init__(self, calibrate_bool):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        self.drift_speed = 0
        self.executor = ThreadPoolExecutor()

        self.franka_running = False  # Reentrancy guard for franka
        self.cartesian_running = False

        self.bridge = CvBridge()

        if calibrate_bool:
            # self.franka_visualize = rospy.Subscriber("/calibrate", Float64, self.franka_visualize)
            
            self.franka_visualize()
            print("Ready to calibrate")


        self.conveyor_speed_sub = rospy.Subscriber('/conveyor_speed', Float64, self.conveyor_speed_callback)

        # self.franka_timer = rospy.Timer(rospy.Duration(5), self.run_franka)
        # rospy.sleep(0.5)
        # self.cartesian_timer = rospy.Timer(rospy.Duration(5), self.run_cartesian)

        self.objects_in_frame = []
        self.objects_we_tried = []

        self.run_franka()
        # self.run_cartesian()

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

    def call_cartesian_robot_service(self, grasps): # WIP
        # Wait for the service to become available
        rospy.wait_for_service('get_plucked')
        try:
            gantry_control = rospy.ServiceProxy('get_plucked', GantrySrv)
            request = GantrySrvRequest()
            request.grasps.data = grasps
            response = gantry_control(request)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to gantry control service failed: {e}")

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


    def franka_visualize(self):

        counter = 0
        while True:
        # while counter <10:
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
                    rospy.loginfo("???")
                else:
                    rospy.logwarn("No grasps received. Exiting run_franka function.")
                    return
                rospy.loginfo("????")
                self.show_grasps(rgb_image, bboxes, grasps)
                counter+=1
                rospy.loginfo(f"Counter = {counter}")
            rospy.loginfo(f"Exit. Counter = {counter}")
            counter = 0
        rospy.loginfo(f"End. Counter = {counter}")


    def show_grasps(self, rgb_image, bboxes, grasps):
        grasps = np.reshape(grasps, (-1, 6))
        bboxes = np.reshape(bboxes, (-1, 5))
        radius = 50          # Radius of the circle
        color = (0, 0, 255)   # Red color in BGR (OpenCV uses BGR, not RGB)
        thickness = 2         # Thickness of the circle's edge (use -1 for a filled circle)
        print(grasps, bboxes)
        # grasps = np.asarray(grasps)
        rospy.loginfo("Getting img")
        rgb_frame = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding="rgb8")
        rospy.loginfo("Got img")

        for bbox in bboxes:
            center = (bbox[1], bbox[2])
            size = (bbox[3], bbox[4])
            half_w = bbox[3]/2.0
            half_h = bbox[4]/2.0

            # min_x = int(bbox[1]-half_w)
            # max_x = int(bbox[1]+half_w)
            # min_y = int(bbox[2]-half_h)
            # max_y = int(bbox[2]+half_h)
            rotated_rect = (center, size, 0)
            
            # Get the four corners of the rotated rectangle
            box_points = cv2.boxPoints(rotated_rect)
            box_points = np.int32(box_points)  # Convert to integer points

            # Draw the rectangle on the image
            cv2.polylines(rgb_frame, [box_points], isClosed=True, color=50, thickness=2)

        ppx=321.1669921875
        ppy=231.57203674316406
        fx=605.622314453125
        fy=605.8401489257812

        for ind, grasp in enumerate(grasps):
            center_z = grasp[2]
            center_x = (grasp[0]/center_z) * fx + ppx
            center_y = (grasp[1]/center_z) * fy + ppy
            print(f"GRASP wrt camera [pixels/pixels/meters]: x: {center_x:0.4f}, y: {center_y:0.4f}, z: {center_z:0.4f}")
            print(f"GRASP wrt camera [meters]: x: {grasp[0]:0.4f}, y: {grasp[1]:0.4f}, z: {grasp[2]:0.4f}")


            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(rgb_frame, f'x: {center_x:0.0f}, y: {center_y:0.0f}, z: {center_z:0.3f}', (20, 20*(ind+1)), font, .3, (0, 255, 0), 1, cv2.LINE_AA)

            center = (int(center_x),int(center_y))
            print(f"center is: {center}, angle is: {grasp[3]*180/3.1415}")
            # print(rgb_image)
            # cv2.circle(rgb_frame, center, radius, color, thickness)
            # cv2.ellipse(img=rgb_frame, center=center, axes=(int(800*grasp[4]), 10), angle=grasp[3]*180/3.1415, startAngle=0, endAngle=360, color=color, thickness=2)

            angle_degrees = grasp[3] * 180 / np.pi
            angle_degrees += 45
            size = (20, 800*grasp[4])  # Size of the rectangle (width, height)
            # Create a rotated rectangle
            rotated_rect = (center, size, angle_degrees)
            
            # Get the four corners of the rotated rectangle
            box_points = cv2.boxPoints(rotated_rect)
            box_points = np.int32(box_points)  # Convert to integer points

            # Draw the rectangle on the image
            cv2.polylines(rgb_frame, [box_points], isClosed=True, color=255, thickness=2)

            cv2.circle(rgb_frame, center, 2, color, 1)

        print("going to show")

        # Display the image with the circle
        cv2.imshow('Live Image', rgb_frame)

        # Press 'q' to exit
        if cv2.waitKey() & 0xFF == ord('q'):
            return -1
        return 0

    def run_franka(self):

        if self.franka_running:
            rospy.loginfo("run_franka callback is already running. Skipping this cycle.")
            return

        self.franka_running = True
        empty_frame = False

        try:
            while not empty_frame:
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
                        empty_frame == True
                        break

                    # Handle grasp inference results
                    if not results[1]:
                        rospy.logwarn("Grasp network's inference failed. Exiting run_franka function.")
                        return

                    rospy.loginfo(f"Results from antipodal grasp service received.")

                else:
                    rospy.logwarn("RGB and Depth Frames did not arrive. Check Service.")
                    return

                # Perform grasp selection
                grasps = self.call_grasp_selection_service(bboxes) # This is a flat array. needs to be reshaped like grasps.reshape(-1, 6) where each
                                                                # row would then become [x, y, z, angle, witdh, label]\
                self.show_grasps(rgb_image, bboxes, grasps)
                grasps_reshaped = np.asarray(grasps).reshape(-1,6)

                #Correct for the insane hand geometry
                grasps_reshaped[:,3] -= (np.pi/4)

                # If this is the first time and there are no objects, add them all
                if len(self.objects_we_tried) == 0:
                    self.objects_in_frame = grasps_reshaped.tolist()

                # Otherwise compare tolerance between x and y points of grasp to current values in list
                else:
                    tolerance = 0.01 #0.03
                    for grasp in grasps_reshaped:
                        if all(abs(x[0] - grasp[0]) > tolerance and abs(x[1] - grasp[1]) > tolerance for x in self.objects_we_tried):
                            self.objects_in_frame.append(grasp.tolist())


                if len(self.objects_in_frame) == 0:
                    empty_frame == True
                    break

                if grasps:
                    rospy.loginfo("Grasps received for given objects.")
                else:
                    rospy.logwarn("No grasps received. Exiting run_franka function.")
                    return

                # Try to execute grasps. All grasp offsets must be handled within the franka/cartesian service.
                # The current grasps are computed at the object, so they need a little offset to not collide with the object.
                msg_list = np.asarray(self.objects_in_frame).flatten()
                rospy.loginfo("Calling franka")
                self.call_franka_robot_service(msg_list)
                rospy.loginfo("Franka called")

                # append to objects we tried to ensure we don't keep trying for the same object
                self.objects_we_tried.append(self.objects_in_frame[0])
                self.objects_in_frame[:] = []
                print(self.objects_in_frame,"HAS THIS BEEN CLEARED?")

            self.objects_we_tried = []
            self.objects_in_frame = []
            print("No more grasps able to be handled by the franka arm")
            self.run_cartesian()
        except Exception as e:
            rospy.logerr(f"Error running run_franka callback: {e}")
        finally:
            self.franka_running = False


    def run_cartesian(self):

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
                                            # grasps.reshape(-1, 4) where each  row would then become [x, y, z, label]

                print("suction grasps collected")
                # Handle suciton grasp results
                if not suction_grasps:
                    rospy.logwarn("Suction grasps not found. Exiting run_cartesian function.")
                    return

                rospy.loginfo(f"Grasps received for given objects.")

                # TODO: add separate integration for conveyor and gantry.
                print("attempting gantry")
                gantry_plucked = self.call_cartesian_robot_service(suction_grasps)

                # self.run_franka()
                print("Done with cartesian")

            else:
                rospy.logwarn("RGB and Depth Frames did not arrive. Check Service.")
                return

            # Try to execute grasps. All grasp offsets must be handled within the franka/cartesian service.
            # The current grasps are computed at the object, so they need a little offset to not collide with the object.
            # self.call_cartesian_robot_service(suction_grasps)

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
