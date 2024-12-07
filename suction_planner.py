#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from suction_gripper_affordance import SuctionGenerator

"""
This class handles translating bounding boxes for use by the manipulation team
"""
class SuctionPlanner:
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("suction_planner")
        self.boxes = []

        # Create subscriber
        rospy.Subscriber('/objects_detected', Float64MultiArray, self.recieve_boxes)
        self.pose_publisher = rospy.Publisher('/cartesian_pose', Float64MultiArray, queue_size=10)

    def generatePose(self):
        # TODO: Call class to calculate and return poses based on bounding boxes
        suction_poses = SuctionGenerator().generate_suction()
        flattened_suction_poses = suction_poses.flatten()

        # Generate message
        msg = Float64MultiArray()
        msg.data = flattened_suction_poses

        rospy.loginfo(f'Suction poses: {msg.data.reshape(-1, 3)}') # x, y, z

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
    SuctionPlanner().run()