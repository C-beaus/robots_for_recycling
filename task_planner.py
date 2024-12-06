#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import argparse
from robots_for_recycling.srv import ClassifySrv, GraspSrv
import pyrealsense2 as rs
import numpy as np
import cv2



class TaskPlanner:
    def __init__(self, manipulator_type, end_effector):
        rospy.init_node("Recycler")
        rospy.sleep(1.0)
        rospy.loginfo("Recycle Node Ready")

        self.classify_publisher = rospy.Publisher('/classify', Float64, queue_size=10)
        rospy.Subscriber('/recycle', Float64, self.main)
        self.manipulator_type = manipulator_type # string
        # rospy.Subscriber

        # # instantiate correct Manipulator object
        # if (manipulator_type == "franka"):
        #     self.manipulator = Manipulator.Franka(end_effector) # TODO: check this once manipulator files are known
        # elif (manipulator_type == "cartesian"): 
        #     self.manipulator = Manipulator.Cartesian(end_effector) # TODO: check this once manipulator files are known
        # else: 
        #     # rospy.logerr("Invalid manipulator type - Valid types are 'franka' and 'cartesian'")
        #     rospy.signal_shutdown("Invalid manipulator type - Valid types are 'franka' and 'cartesian'") # kills node if you typo it 

    def capture_frames(self):
        
        # Create a pipeline
        pipeline = rs.pipeline()

        print("captureing frames")
        # Create a config and enable the bag file
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        pipeline.start(config)

        print('pipeline started')

        # Create an align object to align color frames to depth frames
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        try:
            while True:

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.first(rs.stream.color)

                # Keep looping until valid depth and color frames are received.
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                break
        
        except RuntimeError as e:
            print(f"Error occurred: {e}")
        finally:
            pipeline.stop()
            return color_image
        
    # def draw_bounding_boxes(self, frame, boxes_list): 
    #     for label, center_x, center_y, width, height in boxes_list:
    #         if score > threshold:
    #             xmin, ymin, xmax, ymax = map(int, box)
    #             class_name = self.CLASSES[label]
    #             cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
    #             cv2.putText(frame, f'{class_name}: {score:.2f}', (xmin, ymin - 10),
    #                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    #     return frame

    def main(self, msg):
        # self.classify_publisher.publish(msg)
        objects_detected = True
        while objects_detected:
            rospy.wait_for_service('classify_waste')
            try:
                get_bounding_boxes = rospy.ServiceProxy('classify_waste', ClassifySrv)
                # print("here")
                yoloV5_data = get_bounding_boxes()
                # print(f"here are the boxes: {yoloV5_data}")
                # print(f"type is: {type(yoloV5_data)}")
                # print(f"dir is: {dir(yoloV5_data)}")
                # print(f"output: {yoloV5_data.output}")
                # print(f"output.data {yoloV5_data.output.data}")

                if yoloV5_data.output.data == []:
                    objects_detected = False
                    # print("breaking")
                    break
            except rospy.ServiceException as e:
                print("Classify Service call failed: %s"%e)
            
            # print("done with bboxes, waiting for ")
            rospy.wait_for_service('get_grasps')
            try:
                # print("getting grasps")
                find_grasps = rospy.ServiceProxy('get_grasps', GraspSrv)
                bbox_msg = Float64MultiArray()
                bbox_msg.data = yoloV5_data.output.data

                grasp = find_grasps(bbox_msg)
            except rospy.ServiceException as e:
                print("Grasp Service call failed: %s"%e)


            # print(f"here are the grasps{grasp}")
            rospy.sleep(5)

            try:
                rgb_frame = self.capture_frames()

                # cv2.imshow('Real-Time Waste Detector', rgb_frame)

                # Load an image (replace 'your_image.jpg' with your image file path)
                # image = cv2.imread('your_image.jpg')

                # Define the center of the circle, radius, color (BGR), and thickness
                # center = (250, 250)  # x, y coordinates of the center
                radius = 50          # Radius of the circle
                color = (0, 0, 255)   # Red color in BGR (OpenCV uses BGR, not RGB)
                thickness = 2         # Thickness of the circle's edge (use -1 for a filled circle)

                # Draw the circle on the image

                # print(f"here are the boxes: {grasp}")
                # print(f"type is: {type(yoloV5_data)}")
                # print(f"dir is: {dir(yoloV5_data)}")
                # print(f"output: {yoloV5_data.output}")
                print(f"output.data {grasp.output.data}")

                ppx=321.1669921875
                ppy=231.57203674316406
                fx=605.622314453125
                fy=605.8401489257812

                center_z = grasp.output.data[2]
                center_x = (grasp.output.data[0]/center_z) * fx + ppx
                center_y = (grasp.output.data[1]/center_z) * fy + ppy
                center = (int(center_x),int(center_y))
                print(f"center is: {center}")
                cv2.circle(rgb_frame, center, radius, color, thickness)
                cv2.circle(rgb_frame, center, 2, color, 1)

                print("going to show")

                # Display the image with the circle
                cv2.imshow('Image with Red Circle', rgb_frame)
                rospy.sleep(1)

                # Wait for a key press and close the window
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

                # # Optionally, save the image with the circle
                # cv2.imwrite('image_with_red_circle.jpg', image)
                


                # Press 'q' to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except RuntimeError as e:
                print("Frame missed: %s"%e)
            # frame = self.draw_bounding_boxes(rgb_frame, boxes, labels, scores, threshold=self.confidence_threshold)



            # self.manipulator.pickMove(grasp)

            # # cartesian method should handle moving the conveyor belt to the cartesian robot and then back for further classification and finding best grasp
            # self.manipulator.placeMove(grasp.class_label)
            

            ## Send grasps to mainipulator
            # grasp is formatted as: x y z angle width class_label
            # In switch statement, instantiate subclass of manipulation class based on passed in manipulator argument
            # Also add argument to pass in for type of end effector gripper?
            # Task planner node should also handle talking to conveyor belt to move    




    def run(self):
            rospy.spin()

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

    args = parser.parse_args()

    TaskPlanner(args.manipulator_type, args.end_effector).run()