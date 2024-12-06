#!/usr/bin/env python3

# To run: 

# Terminal 1: cd RoboticRecycling2023/recycling/src   -->  roslaunch RecyclingSystem.launch
# Terminal 2: rosrun rosserial_python serial_node.py _port:=/dev/ttyACM2 _baud:=57600     (Port /dev/ttyACM2 corresponds to Pneumatic Actuator Ardiuno)
# In the Conveyor Belt GUI, in Gantry Controls section, press HOME button. (If this is not done, then the code will not execute)
# Terminal 3: cd RoboticRecycling2023/RBE595/   --> source devel/setup.bash   --> rosrun robots_recycling control_example.py
# If you are doing any chnages then remeber to do 'catkin_make' then 'source devel/setup.bash'

import os
import rospy
import serial
import time
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import * 
from dynamixel_sdk_examples.msg import *
from tool_dynamixel import DynamixelMotor
from std_msgs.msg import Float64, String, Int32, Bool, Float32
# from end_effector.msg import GripperToggle, GripperState
from end_effector_parent import EndEffector

class FinRayGripperControllerChild(EndEffector):
    def __init__(self):#, offset):#, startPos):
        super().__init__() #offset)#May noteed this as its from the person student example
        
        print("TEST")
        #from control_example.py
        self.fin_ray_gripper_command_publisher = rospy.Publisher('/fin_ray_gripper_command_publisher', Bool, queue_size=5)
        self.fin_ray_gripper_command_subscriber = rospy.Subscriber("/fin_ray_gripper_command_subscriber", Bool, self.callback)
        #self.gantry_x_publisher = rospy.Publisher('/SetGantryXPos', Float32, queue_size=5)
        #self.gantry_z_publisher = rospy.Publisher('/SetGantryZPos', Float32,queue_size=5)
        #self.pubPneumatic = rospy.Publisher('/pneumatic_actuator', Bool, queue_size=5)
        #self.pubHome = rospy.Publisher("gantry/Home", Bool, queue_size=1)
        #self.pubX = rospy.Publisher("gantry/X", Float32, queue_size=1)
        #self.pubZ = rospy.Publisher("gantry/Z", Float32, queue_size=1)
        # rospy.Subscriber("/gantryPosZ", String, self.posz_callback)
        # rospy.Subscriber("/gantryPosX", String, self.posx_callback)

    def callback(self, msg):
            if (msg.data):
                self.pick()
            else:
                self.release()
    def pick(self):  # Closes the gripper either to max encoder value or until the max load is hit.
        #I should probably make an alalog version of this.
        msg = Bool()
        msg.data = True

        self.fin_ray_gripper_command_publisher.publish(msg)
        print("sleep 3")
        rospy.sleep(3)
        print("gripper closed")
        self.picked_bool = True
        
    def release(self):
        #report offset
        msg = Bool()
        msg.data = False

        self.fin_ray_gripper_command_publisher.publish(msg)
        print("sleep 3")
        rospy.sleep(3)
        print("gripper opened")   
        self.picked_bool = False 
    

   # def pickedStatusReportFormal(selfPlaceholder):
   #     return f"The {selfPlaceholder} has not grasped nor picked at the presant"    
   # 
   # #def isPicked(self,callback_function_is_picked):  # Closes the gripper either to max encoder value or until the max load is hit
   # def isPickedOld(selfPlaceholder,callback_function_is_picked):  # Closes the gripper either to max encoder value or until the max load is hit
   #     #msg = Bool()
   #     #msg.data = self.picked_val
   #     picked_status_report = callback_function_is_picked(selfPlaceholder)
   #     #isPickedBoolStatus = callback_function_is_picked
   #     #return self.picked_bool
   #     
   #     print(picked_status_report)
   #     #if msg.data == True:
   #     #    return False
   #     #else:
   #     #    return True
    
    #isPicked(selfPlaceholder="Three finger radialy symmetric torsion resistant Fin Ray gripper",callback_function_is_picked=pickedStatusReportFormal)
    ###needs to be moved^^^^
    
    def isPicked(self):  # Closes the gripper either to max encoder value or until the max load is hit
    #    #msg = Bool()
    #    #msg.data = self.picked_val
        return self.picked_bool
    #    #if msg.data == True:
    #    #    return False
    #    #else:
    #    #    return True
    def isReleased(self):
        return not self.picked_bool
        
    def customActuation(self, customActuationValue):
        msg = Float32()
        #msg.data = -customActuationValue
        msg.data = -customActuationValue
        self.gripper_publisher.publish(msg)
        print("Gripper position set to", customActuationValue) 
        #rospy.loginfo(f"Gripper position set to: {pose}")

        rospy.loginfo(f"Franka gripper moving to pose: {customActuationValue}")
        #self.move_group.set_pose_target(pose)
        #self.move_group.go(wait=True)
        #self.pick()
        self.picked_bool = True #may not be true if pose is open position, but user will probably use release in that case

#    def customDisplacementTime(self,time):
#        msg = Float32()
#        msg.data = -time
#        self.gripper_publisher.publish(msg)
#        print("Gripper position set to", time) 
#        #rospy.loginfo(f"Gripper position set to: {pose}")
#
#        rospy.loginfo(f"Franka gripper moving to pose: {time}")
#        #self.move_group.set_pose_target(pose)
#        #self.move_group.go(wait=True)
#        #self.pick()
#        self.picked_bool = True #may not be true if pose is open position, but user will probably use release in that case
#    #add demo function !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#    #main
#
#    #isPicked(selfPlaceholder="Three Fin Ray finger gripper",callback_function_is_picked=pickedStatusReportFormal)
#    #isPicked(selfPlaceholder="Three finger radialy symmetric torsion resistant Fin Ray gripper",callback_function_is_picked=pickedStatusReportFormal)
#    def customDisplacement(self, pose):
#        msg = Float32()
#        msg.data = -pose
#        self.gripper_publisher.publish(msg)
#        print("Gripper position set to", pose) 
#        #rospy.loginfo(f"Gripper position set to: {pose}")
#
#        rospy.loginfo(f"Franka gripper moving to pose: {pose}")
#        #self.move_group.set_pose_target(pose)
#        #self.move_group.go(wait=True)
#        #self.pick()
#        self.picked_bool = True #may not be true if pose is open position, but user will probably use release in that case

def main():
    rospy.init_node('fin_ray_gripper_controller_child')
    #when working with task plannin, uncomment "rospy.spin()" and comment everythiong in main that comes after
    rospy.spin()
    
    #example = FinRayGripperControllerChild()#create this object in a task planner or somewhere that you rcontrolling this
    #rospy.sleep(2)
    #example.pick()
    #rospy.sleep(2)
    #example.release()

if __name__ == '__main__':
    main()