#!/usr/bin/env python3

import rospy
import yaml
import os

'''

'''

def load_yaml(file_path):
    """Load YAML file and set ROS parameters.
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    # Set parameters for camera instrinsics
    rospy.set_param('/camera_instrinsics/fx', data['camera_instrinsics']['fx'])
    rospy.set_param('/camera_instrinsics/fy', data['camera_instrinsics']['fy'])
    rospy.set_param('/camera_instrinsics/cx', data['camera_instrinsics']['cx'])
    rospy.set_param('/camera_instrinsics/cy', data['camera_instrinsics']['cy'])

    # Set transformation matrices
    rospy.set_param('/transformations/camera_to_franka', data['transformations']['camera_to_franka']['matrix'])
    rospy.set_param('/transformations/camera_to_gantry', data['transformations']['camera_to_gantry']['matrix'])
    rospy.set_param('/transformations/camera_to_conveyor_sorting', data['transformations']['camera_to_conveyor_sorting']['matrix'])

if __name__ == "__main__":
    rospy.init_node("yaml_server")

    yaml_path = os.path.join(os.path.dirname(__file__), "config.yaml")
    load_yaml(yaml_path)

    rospy.loginfo("YAML parameters loaded into ROS parameter server.")
    rospy.spin()