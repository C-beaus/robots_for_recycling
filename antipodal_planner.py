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

"""
This class handles translating bounding boxes for use by the manipulation team
"""
class AntipodalPlanner:
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("Antipoldal_Planner")
        self.boxes = []

        # Create subscriber
        rospy.Subscriber('/objects_detected', Float64MultiArray, self.recieve_boxes)
        self.pose_publisher = rospy.Publisher('/franka_pose', Float64MultiArray, queue_size=10)
        self.model_path = '/home/sultan/catkin_ws/src/my_sample_package/scripts/antipodal_grasp_network/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch16/epoch_17_iou_0.96'
        self.generator = GraspGenerator(saved_model_path=self.model_path)
        self.generator.load_model()
        rospy.loginfo(f'Grasp Node Ready.')

    def generatePose(self):
        grasp_poses = generate_poses(generator=self.generator, bboxes=self.boxes, camera2robot=np.eye(4), color_image=None, depth_image=None, use_cam=True)
        flattened_grasp_poses = grasp_poses.flatten()
        msg = Float64MultiArray()
        msg.data = flattened_grasp_poses
        rospy.loginfo(f'Grasp poses: {msg.data.reshape(-1, 5)}')
        # Publish the pose information
        self.pose_publisher.publish(msg)

    def recieve_boxes(self, msg: Float64MultiArray):
        """
        Updates the bounding box information to be used later

        The array msg is formatted as yolo V5 so each array in the list contains class label, center_x, center_y, width, height
        Classes are background, cardboard, glass, metal, paper, plastic in that order so 0 is background, 1 cardboard, etc.
        """
        # Update bounding boxes array
        boxes_array = np.array(msg.data, dtype=np.float64)
        self.boxes = boxes_array.reshape(-1, 5)
        rospy.loginfo(f'Received bounding boxes {self.boxes}')
        # Generate the pose with the bounding boxes
        self.generatePose()


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # Spins until node is terminated        
        rospy.spin()


        
if __name__ == '__main__':
    AntipodalPlanner().run()