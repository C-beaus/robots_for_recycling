#!/usr/bin/env python3

import sys
import numpy as np
from math import pi, cos, sin
import copy
# import moveit_commander.roscpp_initializer
# from moveit_commander.roscpp_initializer import roscpp_initialize
import rospy
import moveit_commander
import tf.transformations as tr
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "src")
print(package_path)

# Set constants for gripper
OPEN = 0.08
CLOSE = 0.0302


class PandaControl():
    def __init__(self) -> None:
        # roscpp_initialize(sys.argv)
        # moveit_commander.roscpp_initializer(sys.argv)
        # rospy.init_node('panda_traj_node',
        #                 anonymous=True)
        # moveit_commander.
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('panda_traj_node',
        #                 anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.moveGroup = moveit_commander.MoveGroupCommander("panda_arm")
        self.moveGroup.allow_replanning(True)
        self.moveGroup.set_planning_time(30.0)
        self.moveGroup.set_num_planning_attempts(20)
        self.gripper = moveit_commander.MoveGroupCommander("panda_hand") 
        self.gripper.set_planning_time(10.0)
        
        self.scene.remove_world_object()
        self.add_coveyor()
        self.add_vbar()
        self.add_hbar()
        self.add_camera()
        self.add_back_wall()
        self.add_side_wall1()
        self.add_side_wall2()
        self.set_def_pos() # Move the robot to home configuartion
        self.set_gripper_distance(0.00)
        print("here")

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
        wpose.orientation.w = posStart[3]
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.moveGroup.compute_cartesian_path(waypoints,0.01, True)  # Generate intermediate waypoints needed for the robot to move to desired location
        
        rospy.sleep(0.2)
        self.moveGroup.execute(plan, wait=True)   # Executes the plan it gets from above
        self.moveGroup.stop()                     # Stops the robot
        return True


    def tf_cam_to_panda(self, pos):
        # Transforms coordiantes from camera frame to panda frame

        p = np.array([pos[0], pos[1], pos[2], 1])

        T = [[0, -1, 0, 0.4719], [1, 0, 0, 0.0535], [0, 0, -1, 0.9953], [0, 0, 0, 1]]

        goal_pos = T@p
        # posPanda = T.dot(xRot).dot(zRot).dot(p)
        return goal_pos
    
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
    
    def move_to_camera_coordinates(self, x, y, z, theta):
        xyzArray = np.array([x, y, z, theta])  # Coodrinates are given wrt to camera    (End effector is not considered as a connected part) 
        pandaCoordinateArray = self.tf_cam_to_panda([xyzArray[0], xyzArray[1], xyzArray[2]])[:-1]   # Convert coordinates to Panda Frame
        
        print("Moving to location: ", round(x,3), ", ", round(y,3), ", ", round(z,3) + " with orientation: ", round(theta,3))
        self.execute_traj_start([pandaCoordinateArray, xyzArray[3]])    # Move the robot to given location
        
    def move_to_home(self):
        self.set_def_pos()
        
    def closeGripper(self, width = -1):
        self.set_gripper_distance(CLOSE if width == -1 else width)
        
    def openGripper(self):
        self.set_gripper_distance(OPEN)

    


def default_test():
    pandaController = PandaControl()

    loaded = np.array([0.0744, -0.0944, 0.8221])  # Coodrinates are given wrt to camera    (End effector is not considered as a connected part) 
    start = pandaController.tf_cam_to_panda(loaded)[:-1]   # Convert coordinates to Panda Frame
    
    print("Moving to Start location")
    pandaController.execute_traj_start(start)    # Move the robot to given location

    # print("Moving to Home location")
    # pandaController.set_def_pos()   # Move the robot to home configuartion
    # pandaController.scene.remove_world_object()   # Remove all the objects from the scene



if __name__ == "__main__":
    default_test()