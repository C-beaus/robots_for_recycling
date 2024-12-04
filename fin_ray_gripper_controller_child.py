#Parent child class examples from: https://www.w3schools.com/python/python_inheritance.asp

#Start if content from control_example.py
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
from end_effector.msg import GripperToggle, GripperState

class FinRayGripperControllerChild(EndEffector):
    def __init__(self, offset, startPos):
        super().__init__(offset)
        self.startPos = startPos
        self.picked_bool = False

    def pick(self):  # Closes the gripper either to max encoder value or until the max load is hit. 
        #I should probably make an alalog version of this.
        msg = Bool()
        msg.data = False

        self.gripper_command_publisher.publish(msg)
        print("gripper closed")
        self.picked_bool = True
        
    def release(self):
        #report offset
        msg = Bool()
        msg.data = True

        self.gripper_command_publisher.publish(msg)
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
