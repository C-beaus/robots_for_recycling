#parent example
#end_effector_controller is a parent class to types of grippers.

#class Person:
#    def __init__(self, fname, lname):
#        self.firstname = fname
#        self.lastname = lname
#
#    def printname(self):
#        print(self.firstname, self.lastname)
#
##Use the Person class to create an object, and then execute the printname method:
#
#x = Person("John", "Doe")
#x.printname() 
#end of parent example

#Child example
#class Student(Person):
#    def __init__(self, fname, lname, year):
#        super().__init__(fname, lname)
#        self.graduationyear = year
#
#    def welcome(self):
#        print("Welcome", self.firstname, self.lastname, "to the class of", self.graduationyear) 
#end of child example

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
#from dynamixel_sdk import *
#from dynamixel_sdk_examples.srv import * 
#from dynamixel_sdk_examples.msg import *
#from tool_dynamixel import DynamixelMotor
from std_msgs.msg import Float64, String, Int32, Bool, Float32  
# from end_effector.msg import GripperToggle, GripperState

class EndEffector:
    def __init__(self):#, offset):
        #self.offset = offset#66.7#offset#needs to be set
        self.picked_bool = False
        
        #CHECK THIS WITH

    ##AB
    #def _pre_grasp_check(self):
    #    rospy.loginfo("Franka gripper pre-grasp check: Validating MoveIt pose...")
    #    # Check if the pose is within the robot's limits or reachable

    def pick(self):  # Closes the gripper either to max encoder value or until the max load is hit. 
        #I should probably make an alalog version of this.
        raise NotImplementedError("This method should be overridden by subclasses")
        

    #AB
    #def _grasp_action(self):
    #    rospy.loginfo("Franka gripper closing...")
    #    # Command Franka's gripper to close
    #    self.move_group.attach_object()

    def release(self):
        #report offset
        raise NotImplementedError("This method should be overridden by subclasses")
    
    #AB
    #def _release_action(self):
    #        rospy.loginfo("Franka gripper opening...")
    #        # Command Franka's gripper to open
    #        self.move_group.detach_object()


    def isPicked(self):  # Closes the gripper either to max encoder value or until the max load is hit
        #msg = Bool()
        #msg.data = self.picked_val
        #return self.picked_bool
        #if msg.data == True:
        #    return False
        #else:
        #    return True
    

        #self.gripper_command_publisher.publish(msg)
        #print("gripper closed")
        raise NotImplementedError("This method should be overridden by subclasses")

    def isReleased(self):
        #return not self.picked_bool
        raise NotImplementedError("This method should be overridden by subclasses")
    
    #def customDisplacement(self)
    #    return not self.picked_bool
    
    def customActuation(self, customActuationValue):
        #msg = Float32()
        #msg.data = -value
        #self.gripper_publisher.publish(msg)
        #print("Gripper position set to", value)
        #pass # I may be using this comand incorrectly and should probably delete this method
        raise NotImplementedError("This method should be overridden by subclasses")

    #AB
    #def pickup(self, pose):
    #    rospy.loginfo(f"Franka gripper moving to pose: {pose}")
    #    self.move_group.set_pose_target(pose)
    #    self.move_group.go(wait=True)
    #    self.grasp()
    
    #def setZposition(self, value):
    #    msg = Float32()
    #    msg.data = -value
    #    self.gantry_z_publisher.publish(msg)
    #    print("z position set to", value)

#def main():
#    isReleased()
#    customDisplacement

#if __name__ == '__main__':
#    main()


#Alex Brattstrom EndEffectorBase
  

#class EndEffectorBase:
#    def __init__(self):
#        pass
#
#    def pickup(self, pose):
#        """Public method to move to a pose and pick up an object."""
#        raise NotImplementedError("This method should be overridden by subclasses")

#    def grasp(self):
#        """Public method to grasp an object."""
#        self._pre_grasp_check()
#        self._grasp_action()
#
#    def release(self):
#        """Public method to release an object."""
#        self._release_action()
#
#    def _pre_grasp_check(self):
#        """Protected method for internal checks before grasping."""
#        raise NotImplementedError("This method should be overridden by subclasses")
#
#    def _grasp_action(self):
#        """Protected method for performing the actual grasp action."""
#        raise NotImplementedError("This method should be overridden by subclasses")

#    def _release_action(self):
#        """Protected method for performing the release action."""
#        raise NotImplementedError("This method should be overridden by subclasses")
#
#
#
#

#
#
#Alex Brattstrom FrankaGripper code
#
#from end_effector_base import EndEffectorBase
#
#class FrankaGripper(EndEffectorBase):
#    def __init__(self, move_group):
#        super().__init__()
#        self.move_group = move_group
#
#    def _pre_grasp_check(self):
#        rospy.loginfo("Franka gripper pre-grasp check: Validating MoveIt pose...")
#        # Check if the pose is within the robot's limits or reachable
#
#    def _grasp_action(self):
#        rospy.loginfo("Franka gripper closing...")
#        # Command Franka's gripper to close
#        self.move_group.attach_object()
#
#    def _release_action(self):
#        rospy.loginfo("Franka gripper opening...")
#        # Command Franka's gripper to open
#        self.move_group.detach_object()
#
#    def pickup(self, pose):
#        rospy.loginfo(f"Franka gripper moving to pose: {pose}")
#        self.move_group.set_pose_target(pose)
#        self.move_group.go(wait=True)
#        self.grasp()

#def callback(a, b):
#    print('Sum = {0}'.format(a+b))
#
#def main(callback=None):
#    print('Add any two digits.')
#    if callback is not None:
#        callback()
#
#tmp_func = lambda: main(lambda: callback(2,3))
#tmp_func()
#
##OR
#
#tmp_func = lambda x,y: main(lambda: callback(x,y))
#tmp_func(2,4)
#
def callback(callback_test_input):
#    #print('Sum = {0}'.format(a+b))
    #print('Is gripper open?: '.format(picked_bool))
    print('callback_test_input: '.format(callback_test_input))
#
def main(callback=None):
    print('calback testing')
    if callback is not None:
        callback()
#
#tmp_func = lambda: main(lambda: callback(2,3))
#tmp_func()
#
#OR
#
#tmp_func = lambda x,y: main(lambda: callback(x,y))
#tmp_func(2,4)