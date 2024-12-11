#!/usr/bin/env python3

import sys
import numpy as np
from math import pi, cos, sin
import copy
import rospy
import moveit_commander
import tf.transformations as tr
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import moveit_msgs.msg
import ipdb
import os
#Should name the folder that the functions you're trying to call are in?
script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "src")
print(package_path)
#package_path = home/merlab/RoboticRecycling2023/RBE595/src/robot_recycling/src
#package_path = os.path.join("src", "src")


class PandaControl():
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_traj_node',
                        anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.moveGroup = moveit_commander.MoveGroupCommander("panda_manipulator")
        self.moveGroup.allow_replanning(True)
        self.moveGroup.set_planning_time(30.0)
        self.moveGroup.set_num_planning_attempts(10)
        self.gripper = moveit_commander.MoveGroupCommander("panda_hand") 
        self.gripper.set_planning_time(10.0)

    def move_joint(self, goalConf):
        # Move the robot on the basis of joint values entered
        conf = self.moveGroup.get_current_joint_values()
        
        conf[0] = goalConf[0]
        conf[1] = goalConf[1]
        conf[2] = goalConf[2]
        conf[3] = goalConf[3]
        conf[4] = goalConf[4]
        conf[5] = goalConf[5]
        conf[6] = goalConf[6]
        rospy.sleep(0.2)
        self.moveGroup.go(conf, wait=True)
        self.moveGroup.stop()


    def move_pose(self, goalPos, goalOrient):
        # Move the robot on the basis of goal pose provided
        pos = geometry_msgs.msg.Pose()

        pos.orientation.x = goalOrient[0]
        pos.orientation.y = goalOrient[1]
        pos.orientation.z = goalOrient[2]
        pos.orientation.w = goalOrient[3]

        pos.position.x = goalPos[0]
        pos.position.y = goalPos[1]
        pos.position.z = goalPos[2]
        self.moveGroup.set_pose_target(pos)
        rospy.sleep(0.2)
        plan = self.moveGroup.go(wait=True)
        self.moveGroup.stop()
        self.moveGroup.clear_pose_targets()


    def get_pose(self):
        # Get current robot pose (Position + Orientation)
        return self.moveGroup.get_current_pose()
    

    def get_joint(self):
        # Get current robot's joint values
        return self.moveGroup.get_current_joint_values()


    def print_current_state(self):
        # Prints current robot state
        print("============ Robot state ============")
        print(self.robot.get_current_state())
        print("")


    def set_def_pos(self):
        # Move the robot to home configuration
        jointDefPos = [-0.46612513339728634, -1.294136674814057, 0.218479638512121, -2.6409009720616643, 0.1999405407877124, 1.3637807440277177, 0.4232816112742533]
        self.move_joint(jointDefPos)
        return True
    

    def execute_traj_start(self, posStart):
        # Move the rbot to desired location
        waypoints = []
        wpose = self.moveGroup.get_current_pose().pose  # Get current robot pose (Position + Orientation)  (End effector is not considered as a connected part. 
                                                        # It gives current pose for joint just before the end effector connection.)
        
        wpose.position.x = posStart[0]
        wpose.position.y = posStart[1]
        wpose.position.z = posStart[2]
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.moveGroup.compute_cartesian_path(waypoints,0.01, True)  # Generate intermediate waypoints needed for the robot to move to desired location
        
        rospy.sleep(0.2)
        self.moveGroup.execute(plan, wait=True)   # Executes the plan it gets from above
        self.moveGroup.stop()                     # Stops the robot
        return True


    def tf_cam_to_panda(self, pos):
        # Transforms coordiantes from camera frame to panda frame

        xRot = np.array([[1, 0, 0, 0],
                        [0, cos(pi), -sin(pi), 0],
                        [0, sin(pi), cos(pi), 0],
                        [0, 0, 0, 1]])

        zRot = np.array([[cos(-pi/2), -sin(-pi/2), 0, 0],
                        [sin(-pi/2), cos(-pi/2), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        
        T = np.array([[1, 0, 0, 0.465],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.75],
                    [0, 0, 0, 1]])

        p = np.array([pos[0], pos[1], pos[2], 1]).T
        posPanda = T.dot(xRot).dot(zRot).dot(p)
        return posPanda
    

    def add_coveyor(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = 0.535
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.045
        box_name = "coveyor"
        scene.add_box(box_name, box_pose, size=(0.85, 1.80, 0.09))
        
        return True


    def add_vbar(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = 0.889
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.51
        box_name = "vbar"
        scene.add_box(box_name, box_pose, size=(0.04, 0.04, 0.84))

        return True
    

    def add_hbar(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = 0.615
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.94
        box_name = "hbar"
        scene.add_box(box_name, box_pose, size=(0.55, 0.04, 0.04))

        return True


    def add_camera(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = 0.505
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.905
        box_name = "camera"
        scene.add_box(box_name, box_pose, size=(0.095, 0.078, 0.050))

        return True
    
    def add_back_wall(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = -0.950
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.5
        box_name = "back_wall"
        scene.add_box(box_name, box_pose, size=(0.05, 2.0, 1.0))

        return True
    
    def add_side_wall1(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = 0.535
        box_pose.pose.position.y = 0.83
        box_pose.pose.position.z = 0.5
        box_name = "side_wall1"   # Another Conveyor Belt
        scene.add_box(box_name, box_pose, size=(1.0, 0.05, 1.0))

        return True
    
    def add_side_wall2(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = 0.535
        box_pose.pose.position.y = -0.63
        box_pose.pose.position.z = 0.5
        box_name = "side_wall2"   # Towards the pick robot
        scene.add_box(box_name, box_pose, size=(1.0, 0.05, 1.0))

        return True
    
    
    def add_object(self, timeout=4):

        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.moveGroup.get_pose_reference_frame()
        box_pose.pose.position.x = 0.515
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.16 + (0.042/2)
        box_name = "block"
        scene.add_box(box_name, box_pose, size=(0.062, 0.03, 0.042))

        return True


    def set_gripper_distance(self, target_distance):

        joint_value = target_distance / 2  # For symmetric grippers, each finger moves half the distance
        self.gripper.set_joint_value_target([joint_value, joint_value])  # Set both finger joints
        self.gripper.go(wait=True)

        return True


def main():
    pandaController = PandaControl()
    
    pandaController.scene.remove_world_object()
    pandaController.add_coveyor()
    pandaController.add_vbar()
    pandaController.add_hbar()
    pandaController.add_camera()
    pandaController.add_back_wall()
    pandaController.add_side_wall1()
    pandaController.add_side_wall2()
    pandaController.set_def_pos() # Move the robot to home configuartion
    pandaController.set_gripper_distance(0.00)
    print("here")

    # while True:
        
        # wpose = pandaController.moveGroup.get_current_pose().pose
        # print(f"x: {wpose.position.x}, y: {wpose.position.y}, z: {wpose.position.z}")

    # loaded = np.array([376, 162, 0.822])

    # ppx=321.1669921875
    # ppy=231.57203674316406
    # fx=605.622314453125
    # fy=605.8401489257812

    # # center_z = grasps[2]
    # # center_x = (grasps[0]/center_z) * fx + ppx
    # # center_y = (grasps[1]/center_z) * fy + ppy

    # R = [[0, 1, 0], [-1, 0, 0], [0, 0, -1]]
    # pR = [0.5663041, 0.1279476, 0.17321596]
    # # pC = [376, 162, 0.822]
    # x = (loaded[0] - ppx) / fx * loaded[2]
    # y = (loaded[1] - ppy) / fy * loaded[2]
    # pC = [x, y, loaded[2]]

    loaded = np.array([0.0744, -0.0944, 0.8221, 1])
    T = [[0, -1, 0, 0.4719], [1, 0, 0, 0.0535], [0, 0, -1, 0.9953], [0, 0, 0, 1]]

    goal_pos = T@loaded

    print(f"goal_pos = {goal_pos}")

    pandaController.execute_traj_start(goal_pos[0:-1]) 
