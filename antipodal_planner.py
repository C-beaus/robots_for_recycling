#!/usr/bin/env python3
import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "antipodal_grasp_network")
if package_path not in sys.path:
    sys.path.append(package_path)
import rospy
from std_msgs.msg import Float64MultiArray
from run_grasp_generator_modified import generate_poses
from inference.grasp_generator_modified import GraspGenerator
import numpy as np
from robots_for_recycling.srv import GraspSrv

"""
This class handles translating bounding boxes for use by the manipulation team
"""
class AntipodalPlanner:
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("Antipodal_Planner")
        self.boxes = []

        self.s = rospy.Service('get_grasps', GraspSrv, self.recieve_boxes)

        # Create subscriber
        rospy.Subscriber('/objects_detected', Float64MultiArray, self.recieve_boxes)
        self.pose_publisher = rospy.Publisher('/franka_pose', Float64MultiArray, queue_size=10)
        current_dir = os.getcwd()
        print(current_dir)
        current_dir = os.path.join(current_dir, "catkin_ws/src/robots_for_recycling") ## b/c catkin workspace is current working directory, append this to front of relative path

        self.model_path = os.path.join(current_dir, 'antipodal_grasp_network/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch16/epoch_17_iou_0.96')
        self.generator = GraspGenerator(saved_model_path=self.model_path)
        self.generator.load_model()
        rospy.loginfo(f'Grasp Node Ready.')

    def generatePose(self):
        grasp_poses = generate_poses(generator=self.generator, bboxes=self.boxes, camera2robot=np.eye(4), color_image=None, depth_image=None, use_cam=True)
        flattened_grasp_poses = grasp_poses.flatten()
        msg = Float64MultiArray()
        msg.data = flattened_grasp_poses
        rospy.loginfo(f'Grasp poses: {msg.data.reshape(-1, 6)}')
        # Publish the pose information
        return msg
        # self.pose_publisher.publish(msg)

    def recieve_boxes(self, msg: Float64MultiArray):
        """
        Updates the bounding box information to be used later

        The array msg is formatted as yolo V5 so each array in the list contains class label, center_x, center_y, width, height
        Classes are background, cardboard, glass, metal, paper, plastic in that order so 0 is background, 1 cardboard, etc.
        """
        # print(f"received message: {msg}")
        # print(f"------ dir: {dir(msg)}")
        # print(f"****** msg.data: {msg.data}")
        # print(f"@@@@@@@@ dir(msg.data) {dir(msg.data)}")
        # print(f"!!!!!!!!!!! msg.data.data {msg.data.data}")
        # Update bounding boxes array
        boxes_array = np.array(msg.data.data, dtype=np.float64)
        self.boxes = boxes_array.reshape(-1, 5)
        rospy.loginfo(f'Received bounding boxes {self.boxes}')
        # Generate the pose with the bounding boxes
        msg_to_send = self.generatePose()
        print(f"msg_to_send {msg_to_send}")
        return msg_to_send


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # Spins until node is terminated        
        rospy.spin()


        
if __name__ == '__main__':
    AntipodalPlanner().run()