#!/usr/bin/env python3

# To run: 

# Terminal 1: cd RoboticRecycling2023/recycling/src   -->  roslaunch RecyclingSystem.launch
# Terminal 2: rosrun rosserial_python serial_node.py _port:=/dev/ttyACM2 _baud:=57600     (Port /dev/ttyACM2 corresponds to Pneumatic Actuator Ardiuno)
# In the Conveyor Belt GUI, in Gantry Controls section, press HOME button. (If this is not done, then the code will not execute)
# Terminal 3: cd RoboticRecycling2023/RBE595/   --> source devel/setup.bash   --> rosrun robots_recycling control_example.py
# If you are doing any changes then remember to do 'catkin_make' then 'source devel/setup.bash'

import os
import rospy
import serial
import time
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import * 
from dynamixel_sdk_examples.msg import *
# from tool_dynamixel import DynamixelMotor
from std_msgs.msg import Float64, String, Int16, Int32, Bool, Float32
# from end_effector.msg import GripperToggle, GripperState
# from fin_ray_gripper_controller_child import *
# from FinRayGripperControllerChild import *
# from fin_ray_gripper_controller_child import *




class ExpClass:
    def __init__(self):
        self.gripper_command_publisher = rospy.Publisher('/gripper_toggle', Bool, queue_size=5)
        self.gantry_x_publisher = rospy.Publisher('/SetGantryXPos', Float32, queue_size=5)
        self.gantry_z_publisher = rospy.Publisher('/SetGantryZPos', Float32,queue_size=5)
        self.pubPneumatic = rospy.Publisher('/pneumatic_actuator', Bool, queue_size=5)
        self.fin_ray_gripper = rospy.Publisher('/fin_ray_gripper_control_child', Bool, queue_size=5)

        self.subHome = rospy.Publisher("/controllers/Home", Bool, queue_size=10)
        self.pubHome = rospy.Publisher("/gantry/Home", Bool, queue_size=1)
        self.pubX = rospy.Publisher("gantry/X", Float32, queue_size=1)
        self.pubZ = rospy.Publisher("gantry/Z", Float32, queue_size=1)

        self.pubConveySpeed = rospy.Publisher('controllers/speed',Int16, queue_size=1) #indicates speed and direction
        self.pubSetConveyor1 = rospy.Publisher('/beltSpeed1', Int16, queue_size=1) # feeder conveyor
        self.pubSetConveyor2 = rospy.Publisher('/beltSpeed2', Int16, queue_size=1) # main conveyor
        self.fin_ray_gripper_command_publisher = rospy.Publisher('/fin_ray_gripper_command_publisher', Bool, queue_size=5)
        self.fin_ray_gripper_command_subscriber = rospy.Subscriber("/fin_ray_gripper_command_subscriber", Bool, self.callback)


        # rospy.Subscriber("/gantryPosZ", String, self.posz_callback)
        # rospy.Subscriber("/gantryPosX", String, self.posx_callback)
        self.currentX = 0
        self.currentZ = 0
        rospy.Subscriber("/gantryPosX", Int16, self.posX_callback)
        rospy.Subscriber("/gantryPosZ", Int16, self.posZ_callback)

        # self.subHomeStatusX = rospy.Subscriber('homingXComplete',Bool,callback=self.setHomeX, queue_size=1)
        # self.subHomeStatusZ = rospy.Subscriber('homingZComplete',Bool,callback=self.setHomeZ, queue_size=1)
        self.is_homing_done = False # If the homing process is done
        self.pubHomeStatus = rospy.Subscriber('master/homeStatus', Bool, self.home_status_callback)


    # Opens the gripper
    def open_gripper(self):
        msg = Bool()
        msg.data = True

        self.gripper_command_publisher.publish(msg)
        print("gripper openend")

    def close_gripper(self):  # Closes the gripper either to max encoder value or until the max load is hit
        msg = Bool()
        msg.data = False

        self.gripper_command_publisher.publish(msg)
        print("gripper closed")

    def callbackHome(self):
        '''
        callback function to bring the gantry to Home.
        Helper Function(s): home_status_callback(self, msg)
        '''
        self.manualX = 0
        self.manualZ = 0
        # self.pubHome.publish(msg.data)

        # Create a Bool message
        # home_msg = Bool()
        # home_msg.data = msg  # Assign the passed boolean value to the message's data attribute
        # print("Publishing")
        msg = Bool()
        msg.data = True
        self.pubHome.publish(msg)
        # self.subHome.publish("True")

        # Wait for the homeing process to complete
        while not self.is_homing_done:
            rospy.loginfo(f"Waiting for the homing process to finish...")
            rospy.sleep(1)
        rospy.loginfo("Gantry homing complete")
    
    def home_status_callback(self, msg):
        '''
        Helper function for callbackHome()
        '''
        self.is_homing_done = msg.data
        rospy.loginfo(f"Home Status: {self.is_homing_done}")


    def callbackManX(self, msg):
        self.manualX = msg.data
        self.pubX.publish(self.manualX)
    
    def callbackManZ(self, msg):
        self.manualZ = msg.data
        self.pubZ.publish(self.manualZ)
    
    def setXposition(self,value):
        target_x = value * 53.3
        msg = Float32()
        msg.data = value
        print("x position set")
        self.gantry_x_publisher.publish(msg) # TODO: Add try-catch

        # Wait for the X-movement process to complete
        while not self.currentX != int(target_x):
            rospy.loginfo(f"Waiting for the X-movement process to finish...")
            rospy.sleep(1)
        rospy.loginfo("Gantry X-direction complete")

        # msg = Float32()
        # msg.data = value
        # self.gantry_x_publisher.publish(msg)
        # print("x position set")

    def posX_callback(self, msg):
        '''
        Helper function for setXposition(int: value)
        '''
        self.currentX = msg.data
        # rospy.loginfo(f"Current X: {int(self.currentX/53.3)}, X value: {self.currentX}")

    def setZposition(self, value):
        msg = Float32()
        msg.data = -value
        self.gantry_z_publisher.publish(msg) # TODO: Add try-catch
        print("z position set to", value)

    def posZ_callback(self, msg):
        '''
        Helper function for setZposition(int: value)
        '''
        self.currentZ = msg.data
        # rospy.loginfo(f"Current Z: {int(self.currentZ/53.3)}, Z value: {self.currentZ}")

    def pneumatic_gripper(self, value):   # Actuate the Gripper: True--> Actuate   False--> Deactuate
        msg = Bool()
        msg.data = value
        self.pubPneumatic.publish(msg)
        if value:
            print("Pneumatic Gripper Actuated")
        print("Pneumatic Gripper De-Actuated")

    def set_conveyor_feeder_speed(self, feeder_speed):
        '''
        Set the speed of the feeder conveyor.
        A positive value is forward. Zero to stop. Negative value is backwards.
        TODO: What are the value limits? (IIRC no more than +/-11?). Should input feeder_speed be only int?
        '''
        try:
            # Create the message for the feeder conveyor speed
            feeder_speed_msg = Int16()
            # feeder_speed_msg.data = feeder_speed
            speed_constant = 8.5 #some hard-coded number from gui.py
            adjusted_speed = feeder_speed * speed_constant
            adjusted_speed = int(adjusted_speed)
            feeder_speed_msg.data = adjusted_speed

            # Publish the speed to the feeder conveyor
            self.pubSetConveyor1.publish(feeder_speed_msg)

            # Log the result
            rospy.loginfo(f"Feeder conveyor speed set to: {feeder_speed}")

        except Exception as e:
            rospy.logerr(f"Failed to set feeder conveyor speed: {e}")

    def set_conveyor_main_speed(self, main_speed):
        '''
        Set the speed of the main conveyor.
        A positive value is forward. Zero to stop. Negative value is backwards.
        TODO: What are the value limits? (IIRC no more than +/-11?). Should input main_speed be only int?
        '''
        try:
            # Create the message for the main conveyor speed
            main_speed_msg = Int16()
            # main_speed_msg.data = main_speed
            speed_constant = 8.5 #some hard-coded number from gui.py
            adjusted_speed = main_speed * speed_constant
            adjusted_speed = int(adjusted_speed)
            main_speed_msg.data = adjusted_speed

            # Publish the speed to the main conveyor
            self.pubSetConveyor2.publish(main_speed_msg)
            # self.pubConveySpeed.publish(main_speed_msg)

            # Log the result
            rospy.loginfo(f"Main conveyor speed set to: {main_speed}")

        except Exception as e:
            rospy.logerr(f"Failed to set main conveyor speed: {e}")

    # def set_conveyor_speeds(main_speed, feeder_speed):
    #     # Wait for the service to become available
    #     rospy.wait_for_service('set_conveyor_speeds')

    #     try:
    #         # Create a service proxy
    #         set_speeds = rospy.ServiceProxy('set_conveyor_speeds', SetConveyorSpeed)

    #         # Call the service with the desired speeds
    #         response = set_speeds(main_speed, feeder_speed)

    #         # Check the response
    #         if response.success:
    #             rospy.loginfo("Conveyor speeds set successfully: " + response.message)
    #         else:
    #             rospy.logerr("Failed to set conveyor speeds: " + response.message)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service call failed: %s" % e)

#Added from fin_ray_gripper_controller_child
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

#where grip location is a 6 array
#grasp_location = [0, 1, 2, 3, 4, 5]
def fin_ray_pick_node(grip_location):
    '''
    Test fin ray gripper
    '''
    rospy.init_node('control_example')               # rospy.sleep has to be passed as the controllers are open looped.
    ex = ExpClass()
    #finray = FinRayGripperControllerChild()
    # wait for the publisher to have at least one subscriber
    while ex.pubHome.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect to /gantry/Home")
        rospy.sleep(0.1)
    print("exp setup finished")
    print("Gantry to Home")
    ex.callbackHome() #necessary!
    #finray.pick(False)   # open the fin ray gripper
    ex.release()   # open the fin ray gripper

    # rospy.sleep(3) #should not be needed
    ex.setXposition(grip_location[0])         # X position of Gantry
    # rospy.sleep(3)  #should not be needed anymore
    ex.setZposition(grip_location[2])          # Z position of Gantry 5
    rospy.sleep(5)
    #finray.pick(True)    # close the fin ray gripper
    ex.pick()    # close the fin ray gripper

    rospy.sleep(3)
    ex.setZposition(1)
    rospy.sleep(2)
    ex.setXposition(2) 
    rospy.sleep(2)
    ex.release()
    rospy.sleep(2)
    #ex.pneumatic_gripper(False)
    print("Done fin_ray_pick_node")
    rospy.sleep(3)

#def pick_node(grip_location):
#    '''
#    Test fin ray gripper
#    '''
#    rospy.init_node('control_example')               # rospy.sleep has to be passed as the controllers are open looped.
#    ex = ExpClass()
#    finray = FinRayGripperControllerChild()
#    # wait for the publisher to have at least one subscriber
#    while ex.pubHome.get_num_connections() == 0:
#        rospy.loginfo("Waiting for subscriber to connect to /gantry/Home")
#        rospy.sleep(0.1)
#    print("exp setup finished")
#    print("Gantry to Home")
#    ex.callbackHome() #necessary!
#    finray.pick(False)   # open the fin ray gripper
#    # rospy.sleep(3) #should not be needed
#    ex.setXposition(grip_location[0])         # X position of Gantry
#    # rospy.sleep(3)  #should not be needed anymore
#    ex.setZposition(grip_location[2])          # Z position of Gantry 5
#    rospy.sleep(5)
#    finray.pick(True)    # close the fin ray gripper
#    rospy.sleep(3)
#    #ex.pneumatic_gripper(False)
#    print("Done")
#    rospy.sleep(3)
def exp_node():
    '''
    Test 3-cup suction gripper
    '''
    rospy.init_node('control_example')               # rospy.sleep has to be passed as the controllers are open looped.
    ex = ExpClass()
    # wait for the publisher to have at least one subscriber
    while ex.pubHome.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect to /gantry/Home")
        rospy.sleep(0.1)
    print("exp setup finished")
    
    print("exp setup finsihsed")
    ex.setZposition(2.0)          # Z position of Gantry
    ex.pneumatic_gripper(False)   # Deactuate the Pneumatic gripper
    rospy.sleep(3)
    ex.setXposition(31.0)         # X position of Gantry
    rospy.sleep(3)
    ex.setZposition(8.0)          # Z position of Gantry 5
    rospy.sleep(5)
    ex.pneumatic_gripper(True)    # Actuate the Pneumatic gripper
    rospy.sleep(5)
    ex.setZposition(2.0)
    rospy.sleep(3)
    ex.setXposition(10.0)
    rospy.sleep(3)
    ex.pneumatic_gripper(False)
    print("Done")
    rospy.sleep(3)


def test_depth():
    rospy.init_node('control_example')
    ex = ExpClass()
    # wait for the publisher to have at least one subscriber
    while ex.pubHome.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect to /gantry/Home")
        rospy.sleep(0.1)
    print("exp setup finished")

    # ex.callbackHome()
    # print("Gantry to home")

    ex.setZposition(2.0)          # Z position of Gantry
    print("ex going to 2cm depth")
    ex.pneumatic_gripper(False)   # Deactuate the Pneumatic gripper
    rospy.sleep(5)

    ex.setXposition(31.0)         # X position of Gantry - approx. middle of conveyor
    rospy.sleep(5)
    ex.setZposition(3.0)          
    print("ex going to 3 depth. units??")
    rospy.sleep(5)
    ex.setZposition(4.0)          
    print("ex going to 4 depth")
    rospy.sleep(5)
    ex.setZposition(5.0)  
    print("ex going to 5 depth")        
    rospy.sleep(5)
    ex.setZposition(6.0)  
    print("ex going to 6 depth")        
    rospy.sleep(5)
    # ex.setZposition(7.0)  
    # print("ex going to 7 depth")        
    # rospy.sleep(5)
    # ex.setZposition(8.0)  
    # print("ex going to 8 depth. kissing conveyor")        
    # rospy.sleep(5)


    ex.setZposition(2.0)  
    print("ex going to 2cm depth")        
    rospy.sleep(2)
    ex.setXposition(10.0)
    print("ex going to 10cm x-pose") 
    rospy.sleep(5)
    print("Done")



def test_home():                             # Need to test if this brings the gantry to Home
    rospy.init_node('control_example')
    ex = ExpClass()
    # wait for the publisher to have at least one subscriber
    while ex.pubHome.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect to /gantry/Home")
        rospy.sleep(0.1)
    print("exp setup finished")
    
    print("Gantry to Home")
    ex.callbackHome()
    # ex.Home_Button_Clicked()

    print("Done")



def test_yale_gripper():
    '''
    This function tests the yale gripper with the watermellon plastic bottle
    '''
    rospy.init_node('control_example')
    ex = ExpClass()
    # wait for the publisher to have at least one subscriber
    while ex.pubHome.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect to /gantry/Home")
        rospy.sleep(0.1)
    print("exp setup finished")

    print("Gantry to Home")
    ex.callbackHome()

    ex.open_gripper()
    rospy.sleep(2)

    ex.setXposition(31.0)   # X position of Gantry - approx. middle of conveyor
    rospy.sleep(5)

    ex.setZposition(5.0)  
    print("ex going to 5 depth")        
    rospy.sleep(5)

    ex.close_gripper()
    rospy.sleep(2)

    print("Gantry to Home")
    ex.callbackHome()

    print("Done")



def test_conveyors():   # Need to test if this changes speed of conveyors
    rospy.init_node('control_example')
    ex = ExpClass()
    # wait for the publisher to have at least one subscriber
    while ex.pubHome.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect to /gantry/Home")
        rospy.sleep(0.1)
    print("exp setup finished")

    print("Gantry to home")
    ex.callbackHome()

    print("gantry to x=31")
    ex.setXposition(31.0)         # X position of Gantry - approx. middle of conveyor
    rospy.sleep(5)

    print("gantry to z=5")
    ex.setZposition(5.0)  
    print("ex going to 5 depth")        
    rospy.sleep(5)

    main_speed = 10
    print("Start main conveyor; speed: ", main_speed)
    ex.set_conveyor_main_speed(main_speed)
    rospy.sleep(3)
    print("Stop main conveyor")
    ex.set_conveyor_main_speed(0)
    rospy.sleep(3)
    print("Done")

    # Set the speeds for main and feeder conveyors
    # set_conveyor_speeds(10, 5)  # Example speeds: 10 for the main conveyor, 5 for the feeder

    

def main():
    grasp_location = [32, 1, 4.5, 3, 4, 5]
    fin_ray_pick_node(grasp_location)
    # exp_node()
    #test_depth() # dont go past 8
    # test_home()
    
    # test_yale_gripper() # test yale gripper with strawberry plastic bottle: homing; go to middle conveyor; grasp; return home; release
    
    # test_conveyors()

if __name__ == '__main__':
    main()
        