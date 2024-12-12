# #!/usr/bin/env python3

# # If you are doing any changes then remember to do 'catkin_make' then 'source devel/setup.bash'
# import sys
# import os
# import rospy
# import serial
# import time
# # from dynamixel_sdk import *
# # from dynamixel_sdk_examples.srv import * 
# # from dynamixel_sdk_examples.msg import *

# from std_msgs.msg import Float64, String, Int16, Int32, Bool, Float32
# # from end_effector.msg import GripperToggle, GripperState
# # from fin_ray_gripper_controller_child import *
# #from FinRayGripperControllerChild import *
# sys.path.append('/home/merl/RoboticRecycling2023/RBE595/src/robots_for_recycling/')

# class ConveyorClass:
#     def __init__(self):
#         self.pubConveySpeed = rospy.Publisher('controllers/speed',Int16, queue_size=1) #indicates speed and direction
#         self.pubSetConveyor1 = rospy.Publisher('/beltSpeed1', Int16, queue_size=1) # feeder conveyor
#         self.pubSetConveyor2 = rospy.Publisher('/beltSpeed2', Int16, queue_size=1) # main conveyor


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