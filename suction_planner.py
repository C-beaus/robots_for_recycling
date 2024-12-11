#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import os, sys
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

from suction_gripper_affordance import SuctionGenerator
from robots_for_recycling.srv import SuctionSrv, SuctionSrvResponse
from cv_bridge import CvBridge 

"""
This class handles translating bounding boxes for use by the manipulation team
"""

class SuctionPlanner:
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("suction_planner_service")

        self.cam2robot = np.eye(4) # Replace with the extrinsic calibration
        self.grid_resoltuion = 0.001 # Forgot what value was good for this.
        self.coverage_threshold = 0.85
        self.belt_depth = 0.804 # Measure this, I do not remember correct value
        self.z_component_threshold = 0.9 # For surface normals.
        self.succ_gen = SuctionGenerator(self.cam2robot, self.grid_resoltuion, self.coverage_threshold, self.belt_depth)
        self.boxes = None
        self.bridge = CvBridge()
        
        # Create service
        self.s = rospy.Service('get_sucked', SuctionSrv, self.receive_data)
        rospy.loginfo(f'Suction Node Ready.')

    def generate_suction_grasps(self):

        suction_poses = self.succ_gen.generate_suction(self.depth_map, self.boxes, self.z_component_threshold) # [[x, y, z, label], .....]
        flattened_suction_poses = suction_poses.flatten()

        return flattened_suction_poses

    def receive_data(self, req):
        
        bboxes = np.array(req.bbs.data)
        self.boxes = bboxes.reshape(-1, 5)
        # self.depth_map = req.depth_image
        self.depth_map = np.copy(self.bridge.imgmsg_to_cv2(req.depth_image, desired_encoding="passthrough"))

        rospy.loginfo(f'Received bounding boxes {self.boxes}')

        # Generate the pose with the bounding boxes
        flattened_suction_poses = self.generate_suction_grasps()

        # Generate response
        response = SuctionSrvResponse()
        response.grasps.data = flattened_suction_poses

        rospy.loginfo(f'Suction poses: {response.grasps.data.reshape(-1, 4)}') # x, y, z, label

        return response


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # Spins until node is terminated        
        rospy.spin()


        
if __name__ == '__main__':
    SuctionPlanner().run()