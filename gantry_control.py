# #!/usr/bin/env python3

# # To run: 

# # Terminal 1: cd RoboticRecycling2023/recycling/src   -->  roslaunch RecyclingSystem.launch
# # Terminal 2: rosrun rosserial_python serial_node.py _port:=/dev/ttyACM2 _baud:=57600     (Port /dev/ttyACM2 corresponds to Pneumatic Actuator Ardiuno)
# #       NOTE: ttyACM2 or ttyACM3 may refer to the USB port on top of the tower (where the camera is connected)
# # In the Conveyor Belt GUI, in Gantry Controls section, press HOME button. (If this is not done, then the code will not execute)
# #       NOTE: This action can be automated with callbackHome(self)
# # Terminal 3: cd RoboticRecycling2023/RBE595/   --> source devel/setup.bash   --> rosrun robots_recycling control_example.py

# # If you are doing any changes then remember to do 'catkin_make' then 'source devel/setup.bash'
# import sys
# import os
# import rospy
# import serial
# import time
# from dynamixel_sdk import *
# from dynamixel_sdk_examples.srv import * 
# from dynamixel_sdk_examples.msg import *

# from std_msgs.msg import Float64, String, Int16, Int32, Bool, Float32
# # from end_effector.msg import GripperToggle, GripperState
# # from fin_ray_gripper_controller_child import *
# #from FinRayGripperControllerChild import *
# sys.path.append('/home/merl/RoboticRecycling2023/RBE595/src/robots_for_recycling/')
# from fin_ray_gripper_controller_child import FinRayGripperControllerChild


# class GantryClass:
#     def __init__(self):
#         self.gripper_command_publisher = rospy.Publisher('/gripper_toggle', Bool, queue_size=5)
#         self.gantry_x_publisher = rospy.Publisher('/SetGantryXPos', Float32, queue_size=5)
#         self.gantry_z_publisher = rospy.Publisher('/SetGantryZPos', Float32,queue_size=5)
#         self.pubPneumatic = rospy.Publisher('/pneumatic_actuator', Bool, queue_size=5)
#         self.fin_ray_gripper = rospy.Publisher('/fin_ray_gripper_control_child', Bool, queue_size=5)

#         self.subHome = rospy.Publisher("/controllers/Home", Bool, queue_size=10)
#         self.pubHome = rospy.Publisher("/gantry/Home", Bool, queue_size=1)
#         self.pubX = rospy.Publisher("gantry/X", Float32, queue_size=1)
#         self.pubZ = rospy.Publisher("gantry/Z", Float32, queue_size=1)

#         self.fin_ray_gripper_command_publisher = rospy.Publisher('/fin_ray_gripper_command_publisher', Bool, queue_size=5)
#         self.fin_ray_gripper_command_subscriber = rospy.Subscriber("/fin_ray_gripper_command_subscriber", Bool, self.callback)

#         # rospy.Subscriber("/gantryPosZ", String, self.posz_callback)
#         # rospy.Subscriber("/gantryPosX", String, self.posx_callback)
        
#         rospy.Subscriber("/gantryPosX", Int16, self.posX_callback)
#         rospy.Subscriber("/gantryPosZ", Int16, self.posZ_callback)
#         self.currentX = 0
#         self.currentZ = 0
#         self.z_mapping = {
#             0: -500,
#             1: -1000,
#             2: -2000,
#             3: -3000,
#             4: -4000,
#             5: -5000,
#             6: -6000,
#             7: -7000,
#             8: -8000,
#             9: -9000
#         }

#         # self.subHomeStatusX = rospy.Subscriber('homingXComplete',Bool,callback=self.setHomeX, queue_size=1)
#         # self.subHomeStatusZ = rospy.Subscriber('homingZComplete',Bool,callback=self.setHomeZ, queue_size=1)
#         self.is_homing_done = False # If the homing process is done
#         self.pubHomeStatus = rospy.Subscriber('master/homeStatus', Bool, self.home_status_callback)

#     def callback(self, msg):
#             if (msg.data):
#                 self.pick()
#             else:
#                 self.release()
    
#     def open_gripper(self):
#         '''
#         Opens the yale gripper
#         '''
#         msg = Bool()
#         msg.data = True

#         self.gripper_command_publisher.publish(msg)
#         print("gripper openend")

#     def close_gripper(self):
#         '''
#         Closes the gripper either to max encoder value or until the max load is hit
#         '''
#         msg = Bool()
#         msg.data = False

#         self.gripper_command_publisher.publish(msg)
#         print("gripper closed")

#     def callbackHome(self):
#         '''
#         callback function to bring the gantry to Home.
#         Helper Function(s): home_status_callback(self, msg)
#         '''
#         self.manualX = 0
#         self.manualZ = 0
#         # self.pubHome.publish(msg.data)

#         # Create a Bool message
#         # home_msg = Bool()
#         # home_msg.data = msg  # Assign the passed boolean value to the message's data attribute
#         # print("Publishing")
#         msg = Bool()
#         msg.data = True
#         self.pubHome.publish(msg)
#         # self.subHome.publish("True")

#         # Wait for the homeing process to complete
#         while not self.is_homing_done:
#             rospy.loginfo(f"Waiting for the homing process to finish...")
#             rospy.sleep(1)
#         rospy.loginfo("Gantry homing complete")
    
#     def home_status_callback(self, msg):
#         '''
#         Helper function for callbackHome()
#         '''
#         self.is_homing_done = msg.data
#         rospy.loginfo(f"Home Status: {self.is_homing_done}")


#     def callbackManX(self, msg):
#         self.manualX = msg.data
#         self.pubX.publish(self.manualX)
    
#     def callbackManZ(self, msg):
#         self.manualZ = msg.data
#         self.pubZ.publish(self.manualZ)
    
#     def setXposition(self,value):
#         '''
#         # NOTE: The while loop may introduce problems if the gantry does not get exactly to its target position
#         '''
#         target_X = value * 53.3
#         msg = Float32()
#         msg.data = value
#         # rospy.loginfo(f"Gantry X position set to {value}")
#         self.gantry_x_publisher.publish(msg) # TODO: Add try-catch

#         # Wait for the X-movement process to complete
#         while self.currentX != int(target_X):
#             # rospy.loginfo(f"Waiting for the X-movement process to finish...")
#             # rospy.loginfo(f"Grantry - Set X: {value}, Current X: {self.currentX}, Target X : {target_X}")
#             rospy.sleep(1)
#         rospy.loginfo("Gantry X-direction complete")

#         # msg = Float32()
#         # msg.data = value
#         # self.gantry_x_publisher.publish(msg)
#         # print("x position set")

#     def posX_callback(self, msg):
#         '''
#         Helper function for setXposition(int: value)
#         '''
#         self.currentX = msg.data
#         # rospy.loginfo(f"Current X: {int(self.currentX/53.3)}, X value: {self.currentX}")

#     def setZposition(self, value):
#         '''
#         # NOTE: The while loop may introduce problems if the gantry does not get exactly to its target position
#         '''
#         msg = Float32()
#         msg.data = -value
#         rospy.loginfo(f"Gantry Z position set to {value}")
#         self.gantry_z_publisher.publish(msg)

#         # Wait for the Z-movement process to complete
#         target_Z = self.z_mapping[value]
#         while self.currentZ != int(target_Z): 
#             rospy.loginfo(f"Gantry - Set Z: {value}, Current Z: {self.currentZ}, Target Z : {target_Z}")
#             rospy.sleep(1)
#         rospy.loginfo("Gantry Z-direction complete")
#         # self.gantry_z_publisher.publish(msg) # TODO: Add try-catch
        

#     def posZ_callback(self, msg):
#         '''
#         Helper function for setZposition(int: value)
#         '''
#         self.currentZ = msg.data
#         # rospy.loginfo(f"Current Z: {int(self.currentZ/53.3)}, Z value: {self.currentZ}")

#     def pneumatic_gripper(self, value):   # Actuate the Gripper: True--> Actuate   False--> Deactuate
#         msg = Bool()
#         msg.data = value
#         self.pubPneumatic.publish(msg)
#         if value:
#             print("Pneumatic Gripper Actuated")
#         print("Pneumatic Gripper De-Actuated")

#     def set_conveyor_feeder_speed(self, feeder_speed):
#         '''
#         Set the speed of the feeder conveyor.
#         A positive value is forward. Zero to stop. Negative value is backwards.
#         TODO: What are the value limits? (IIRC no more than +/-11?). Should input feeder_speed be only int?
#         '''
#         try:
#             # Create the message for the feeder conveyor speed
#             feeder_speed_msg = Int16()
#             # feeder_speed_msg.data = feeder_speed
#             speed_constant = 8.5 #some hard-coded number from gui.py
#             adjusted_speed = feeder_speed * speed_constant
#             adjusted_speed = int(adjusted_speed)
#             feeder_speed_msg.data = adjusted_speed

#             # Publish the speed to the feeder conveyor
#             self.pubSetConveyor1.publish(feeder_speed_msg)

#             # Log the result
#             rospy.loginfo(f"Feeder conveyor speed set to: {feeder_speed}")

#         except Exception as e:
#             rospy.logerr(f"Failed to set feeder conveyor speed: {e}")

#     def set_conveyor_main_speed(self, main_speed):
#         '''
#         Set the speed of the main conveyor.
#         A positive value is forward. Zero to stop. Negative value is backwards.
#         TODO: What are the value limits? (IIRC no more than +/-11?). Should input main_speed be only int?
#         '''
#         try:
#             # Create the message for the main conveyor speed
#             main_speed_msg = Int16()
#             # main_speed_msg.data = main_speed
#             speed_constant = 8.5 #some hard-coded number from gui.py
#             adjusted_speed = main_speed * speed_constant
#             adjusted_speed = int(adjusted_speed)
#             main_speed_msg.data = adjusted_speed

#             # Publish the speed to the main conveyor
#             self.pubSetConveyor2.publish(main_speed_msg)
#             # self.pubConveySpeed.publish(main_speed_msg)

#             # Log the result
#             rospy.loginfo(f"Main conveyor speed set to: {main_speed}")

#         except Exception as e:
#             rospy.logerr(f"Failed to set main conveyor speed: {e}")

#     # def set_conveyor_speeds(main_speed, feeder_speed):
#     #     # Wait for the service to become available
#     #     rospy.wait_for_service('set_conveyor_speeds')

#     #     try:
#     #         # Create a service proxy
#     #         set_speeds = rospy.ServiceProxy('set_conveyor_speeds', SetConveyorSpeed)

#     #         # Call the service with the desired speeds
#     #         response = set_speeds(main_speed, feeder_speed)

#     #         # Check the response
#     #         if response.success:
#     #             rospy.loginfo("Conveyor speeds set successfully: " + response.message)
#     #         else:
#     #             rospy.logerr("Failed to set conveyor speeds: " + response.message)
#     #     except rospy.ServiceException as e:
#     #         rospy.logerr("Service call failed: %s" % e)