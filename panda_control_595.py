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
from panda_hw.srv import PandaSrv, PandaSrvResponse
import os
from franka_gripper.msg import GraspActionGoal
from scipy.spatial.transform import Rotation as R
import quaternion



script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "src")
print(package_path)

# Set constants for gripper
OPEN = 0.08
CLOSE = 0.00 #0.0302


class PandaControl():
    def __init__(self) -> None:
        # roscpp_initialize(sys.argv)
        # moveit_commander.roscpp_initializer(sys.argv)
        rospy.init_node('panda_traj_node')
        rospy.sleep(1.0)
        rospy.loginfo("Panda Control Node Ready")

        # moveit_commander.
        moveit_commander.roscpp_initialize(sys.argv)
        self.grasp_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1) 
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
        # self.grasp(OPEN)
        self.grasp(OPEN)
        # rospy.sleep(5)
        # self.grasp(CLOSE)
        print("here")

        self.home_quat = np.quaternion(0.9236308294044129, -0.3832040671747972, 0.0034829607401068493, 0.006971575065411934)

        # Define recycling locations
        self.plastic_location = np.array([0.00, -0.3, 0.500, 0.008760752531346035])
        self.cardboard_location = np.array([0.00, 0.3, 0.500, 0.008760752531346035])
        #TODO: Implement locations for the other 3 classes
       
        self.s = rospy.Service("panda_control", PandaSrv, self.main)

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
        print(f"wpose orientation: {wpose.orientation}")
        wpose.position.x = posStart[0]
        wpose.position.y = posStart[1]
        wpose.position.z = posStart[2]


        # T = np.array([[1, 0, 0, 0.5052], [0, -1, 0, 0.0872], [0, 0, -1, 1.0007], [0, 0, 0, 1]])
        # T = np.array([[-0.07870932,  0.97962212, -0.18478459,  0.42135239,], [0.99158377,  0.09604609,  0.08681456,  0.1636242], [0.10279331, -0.17639629, -0.97893712,  3.49994303], [0, 0, 0, 1]])
        # T = np.array([[-0.15221508, -0.08804008,  0.98441836,  0.22514847], [ 0.96842411 ,-0.21226639 , 0.13075823,  0.14772686], [ 0.19744697,  0.97323786,  0.11757028,  3.06855021], [ 0, 0, 0, 1]])
        # T = np.array([[ 0.98154611, 0.06768895, 0.17884473, 0.02177584],[ 0.06980418,-0.99754525,-0.00555361, 0.24988011],[ 0.17802979, 0.01793524,-0.98386164, 3.60822785],[ 0.        , 0.        , 0.        , 1.        ]])
        # T = np.array([[ 0.96692704, 0.02153965, 0.25414199, 0.42230082],[ 0.0459101 ,-0.99485087,-0.09035491, 0.12896164],[ 0.25088717, 0.09903429,-0.96293709, 0.90417563],[ 0.        , 0.        , 0.        , 1.        ]])
        T = np.array([[ 1, 0, 0, 0.42230082],[ 0 ,-1,0, 0.12896164],[ 0, 0,-1, 0.90417563],[ 0.        , 0.        , 0.        , 1.        ]])
        print(f"target angle before: {posStart[3]}")

        R_cam2base = T[:3, :3]

        r = R.from_euler('z', posStart[3] ,degrees=False)

        r_test = R.from_euler('z', np.pi)
        print(f"r = {r.as_matrix()}")
        home_rot = R.from_quat([self.home_quat.x, self.home_quat.y, self.home_quat.z, self.home_quat.w])

        R_angle2cam = r.as_matrix()
        # R_base2ee = home_rot.as_matrix()

        goal_rot_mat = (R_angle2cam @ R_cam2base)

        R_object = R.from_matrix(goal_rot_mat)

        quaternion = R_object.as_quat()

        # # Convert camera to robot angle
        # angle = np.asarray([0, 0, posStart[3]])
        # angle.shape = (3, 1)
        # R_transform = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        # target_angle = np.dot(R_transform, angle)
        # ###################################################3

        # # angle to quaternion
        # r = R.from_euler('z', posStart[3])  # 'z' denotes the axis of rotation\
        # q1 = r.as_quat()  # Returns the quaternion as [x, y, z, w]

        # # Step 1: Define the quaternion q1 (in reference frame F1)
        # # q1 = np.array([0.707, 0, 0, 0.707])  # Example quaternion (rotation of 90 degrees around Z-axis)

        # # Step 2: Convert quaternion q1 to a rotation matrix (R_q)
        # r1 = R.from_quat(q1)  # Convert quaternion to Rotation object
        # R_q = r1.as_matrix()  # Rotation matrix corresponding to q1

        # # Step 3: Define the rotation matrix R (transforms F1 to F2)
        # # Example: 90 degrees around the X-axis
        # R_transform = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        # # Step 4: Apply the transformation R to the rotation matrix R_q
        # R_2 = R_q @ R_transform
        # # R_2 = np.dot(R_transform, np.dot(R_q, R_transform.T))

        # # Step 5: Convert the resulting rotation matrix R_2 back to a quaternion
        # r2 = R.from_matrix(R_2)
        # quaternion = r2.as_quat()  # Quaternion in reference frame F2

        

        # r2 = R.from_euler('z',-target_angle[2])
        # q2 = r2.as_quat()[0]



        # q1 = np.array([wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w])
        # quaternion = np.dot(q2, q1)
        # print(f"quaternion = {quaternion}")


        # q1 = np.quaternion(wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w)

        # q2_new = np.quaternion(q2[0], q2[1], q2[2], q2[3])
        # quaternion = q1 * q2_new
        # quaternion = self.home_quat * q2_new

        # quaternion = q1 * q2_new

        # print(f"q2_new = {q2_new}")
        print(f"result = {quaternion}")
        





        # r2 = R.from_euler('z', 0)
        # r3 = R.from_euler('z', angle[2])
        # # quaternion = r2.as_quat()[0]
        # quaternion = r2.as_quat()
        # quaternion2 = r3.as_quat()
        # print(f"quaternion 2 = {quaternion2}")

        # quaternion = [0, 0, , target_angle[2]]

        # print(f"target angle: {target_angle}")
        # print(f"quaternion: {quaternion}")

        # wpose.orientation.x = q2_new.x
        # wpose.orientation.y = q2_new.y
        # wpose.orientation.z = -q2_new.z
        # wpose.orientation.w = q2_new.w


        # # wpose.orientation.w = posStart[3]
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.moveGroup.compute_cartesian_path(waypoints,0.01, True)  # Generate intermediate waypoints needed for the robot to move to desired location
        
        rospy.sleep(0.2)
        self.moveGroup.execute(plan, wait=True)   # Executes the plan it gets from above
        self.moveGroup.stop()                     # Stops the robot
        return True


    def tf_cam_to_panda(self, pos):
        # Transforms coordiantes from camera frame to panda frame

        p = np.array([pos[0], pos[1], pos[2], 1])

        # T = [[0, -1, 0, 0.4719], [1, 0, 0, 0.0535], [0, 0, -1, 0.9953], [0, 0, 0, 1]]
        # T = [[0, -1, 0, 0.6101], [1, 0, 0, -0.0176], [0, 0, -1, 1.0007], [0, 0, 0, 1]]
        # T = [[1, 0, 0, 0.5052], [0, -1, 0, 0.0872], [0, 0, -1, 1.0007], [0, 0, 0, 1]]

        T = [[1, 0, 0, 0.531952954758078], [0, -1, 0, 0.120440603833951], [0, 0, -1, 0.929490745042573], [0, 0, 0, 1]]


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


    # def set_gripper_distance(self, target_distance):

    #     try:
    #         joint_value = target_distance / 2  # For symmetric grippers, each finger moves half the distance
    #         self.gripper.set_joint_value_target([joint_value, joint_value])  # Set both finger joints
    #         self.gripper.go(wait=True)
    #     except:
    #         print("gripper traj failed")

    #     return True
    
    def move_to_camera_coordinates(self, x, y, z, theta):
        xyzArray = np.array([x, y, z, theta])  # Coodrinates are given wrt to camera    (End effector is not considered as a connected part) 
        pandaCoordinateArray = self.tf_cam_to_panda([xyzArray[0], xyzArray[1], xyzArray[2]])[:-1]   # Convert coordinates to Panda Frame
        
        print("Moving to location: ", round(x,3), ", ", round(y,3), ", ", round(z,3) + " with orientation: ", round(theta,3))
        self.execute_traj_start([pandaCoordinateArray, xyzArray[3]])    # Move the robot to given location
        
    def move_to_home(self):
        self.set_def_pos()
        
    # def closeGripper(self, width = -1):
    #     self.set_gripper_distance(CLOSE if width == -1 else width)
        
    # def openGripper(self):
    #     self.set_gripper_distance(OPEN)

    def grasp(self, width):

        grasp_data = GraspActionGoal()
        grasp_data.goal.width = width
        grasp_data.goal.force = 0.7
        grasp_data.goal.speed = 0.2
        grasp_data.goal.epsilon.inner = 0.5
        grasp_data.goal.epsilon.outer = 0.5

        self.grasp_pub.publish(grasp_data)
        rospy.sleep(0.5)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down panda control node")
        

    
    def main(self, msg):
        # print(msg)
        grasp = msg.grasps.data
        print(f"grasps received {grasp}")

        width_offset = 0.005 # to subtract from gripper width
        height_offset = 0.09 # to add to robot z height

        # ppx=321.1669921875
        # ppy=231.57203674316406
        # fx=605.622314453125
        # fy=605.8401489257812

        # center_z = grasp[2]
        # center_x = (grasp[0]/center_z) * fx + ppx
        # center_y = (grasp[1]/center_z) * fy + ppy
        # width_in_meters = grasp[4] * (grasp[2]/
        width_in_meters = grasp[4] #+ 0.01

        base_orientation = 0.008760752531346035
        orientation = grasp[3]

        if width_in_meters <= OPEN:
            try:
                # pandaController = PandaControl()

                # move to above object
                loaded = np.array([grasp[0], grasp[1], grasp[2]]) # -0.2
                start = self.tf_cam_to_panda(loaded)[:-1]   # Convert coordinates to Panda Frame
                # start2 = np.array(start)

                start = np.array([start[0], start[1], start[2] + 0.2, orientation])

                try: 
                    print("Moving to Grasp location")
                    self.execute_traj_start(start)
                except rospy.ROSException as e:
                    rospy.loginfo(f"Error while moving to start position: {e}")

                # move arm so gripper is inside
                print("should be above grasp, moving gripper now")
                loaded = np.array([grasp[0], grasp[1], grasp[2]])
                within_gripper = self.tf_cam_to_panda(loaded)[:-1]   # Convert coordinates to Panda Frame
                # within_gripper2 = np.array(within_gripper)


                within_gripper = np.array([within_gripper[0], within_gripper[1], within_gripper[2] + height_offset, orientation])
                
                print("Moving to Within Gripper")
                self.execute_traj_start(within_gripper)

                # close the gripper around the object
                self.grasp(width_in_meters - width_offset)

                # move back to above object                
                print("Moving to Above location")
                self.execute_traj_start(start)

                # move to home configuration
                print("Moving to home")
                waypoint = np.array([0.5, -0.3, 0.500, 0.008760752531346035])
                self.execute_traj_start(waypoint)

                # create switch statement for which location to go to based on the label we are receiving
                self.execute_traj_start(self.plastic_location)

                # open gripper
                self.grasp(OPEN)

                # Move back to home
                print("Moving to home")
                self.set_def_pos()

                response = PandaSrvResponse()
                response.flag.data = 1

                return response
            except rospy.ROSException as e:
                rospy.loginfo("Can not pick up object because width of grasp is too wide")
                rospy.loginfo(f"Error is : {e}")
                response = PandaSrvResponse()
                response.flag.data = 0

                return response
        else:
            print("not getting any viable grasps")



def default_test():
    pandaController = PandaControl()

    loaded = np.array([0.0744, -0.0944, 0.8221])  # Coodrinates are given wrt to camera    (End effector is not considered as a connected part) 
    start = pandaController.tf_cam_to_panda(loaded)[:-1]   # Convert coordinates to Panda Frame

    home_pos = np.array([start[0], start[1], start[2]]) #0.00
    
    print("Moving to Start location")
    pandaController.execute_traj_start(home_pos)    # Move the robot to given location

    # print("Moving to Home location")
    # pandaController.set_def_pos()   # Move the robot to home configuartion
    # pandaController.scene.remove_world_object()   # Remove all the objects from the scene



if __name__ == "__main__":
    PandaControl().run()
    # default_test()