#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import os, sys
import serial
import time
from std_msgs.msg import Float64, String, Int16, Int32, Bool, Float32
from robots_for_recycling.srv import GantrySrv

'''
Terminal command to test service call:
$ rosservice call /get_plucked ""


'''

class GantryControl:
    """
    Service to control the gantry and conveyor for use with the 3-cup suction manipulator. 
    Only works with one item at a time on the main conveyor. Does not handle multiple items.
    TODO: Create and integrate service into project.
    TODO: Create and/or integrate GantrySrv file.
    TODO: Edit conveyor speed units. Create calibration.
    TODO: Add support for varying conveyor speeds.
    TODO: Add support for multiple items.
    """
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("gantry_control_service")
        self.useConveyor = True
        self.cam2robot = np.eye(4) # Replace with the extrinsic calibration

        self.cam2gantryHome_x = 0.7750 # [meters]
        self.cam2gantryHome_y = 0.955  # [meters]
        self.cam2gantryHome_z = -0.620 # [meters]

        self.conveyor_speed = 10  # Default speed in... [cm/s]???

        self.z_component_threshold = 0.8 # For surface normals.
        
        self.gripper_command_publisher = rospy.Publisher('/gripper_toggle', Bool, queue_size=5)
        self.gantry_x_publisher = rospy.Publisher('/SetGantryXPos', Float32, queue_size=5)
        self.gantry_z_publisher = rospy.Publisher('/SetGantryZPos', Float32,queue_size=5)
        self.pubPneumatic = rospy.Publisher('/pneumatic_actuator', Bool, queue_size=5)

        self.subHome = rospy.Publisher("/controllers/Home", Bool, queue_size=10)
        self.pubHome = rospy.Publisher("/gantry/Home", Bool, queue_size=1)
        self.pubX = rospy.Publisher("gantry/X", Float32, queue_size=1)
        self.pubZ = rospy.Publisher("gantry/Z", Float32, queue_size=1)

        self.pubConveySpeed = rospy.Publisher('controllers/speed',Int16, queue_size=1) #indicates speed and direction
        self.pubSetConveyor1 = rospy.Publisher('/beltSpeed1', Int16, queue_size=1) # feeder conveyor
        self.pubSetConveyor2 = rospy.Publisher('/beltSpeed2', Int16, queue_size=1) # main conveyor
        # rospy.Subscriber("/gantryPosZ", String, self.posz_callback)
        # rospy.Subscriber("/gantryPosX", String, self.posx_callback)
        
        rospy.Subscriber("/gantryPosX", Int16, self.posX_callback)
        rospy.Subscriber("/gantryPosZ", Int16, self.posZ_callback)
        self.currentX = 0
        self.currentZ = 0

        self.z_mapping = {
            0: -500,
            1: -1000,
            2: -2000,
            3: -3000,
            4: -4000,
            5: -5000,
            6: -6000,
            7: -7000,
            8: -8000,
            9: -9000
        }

        # Create service
        self.s = rospy.Service('get_plucked', GantrySrv, self.receive_data) 
        rospy.loginfo(f'Gantry Node Ready.')

        self.is_homing_done = False # If the homing process is done
        self.pubHomeStatus = rospy.Subscriber('master/homeStatus', Bool, self.home_status_callback)

    def callback(self, msg):
            if (msg.data):
                self.pick()
            else:
                self.release()

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
        '''
        # NOTE: The while loop may introduce problems if the gantry does not get exactly to its target position
        '''
        target_X = value * 53.3
        msg = Float32()
        msg.data = value
        # rospy.loginfo(f"Gantry X position set to {value}")
        self.gantry_x_publisher.publish(msg) # TODO: Add try-catch

        # Wait for the X-movement process to complete
        while self.currentX != int(target_X):
            # rospy.loginfo(f"Waiting for the X-movement process to finish...")
            # rospy.loginfo(f"Grantry - Set X: {value}, Current X: {self.currentX}, Target X : {target_X}")
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
        '''
        # NOTE: The while loop may introduce problems if the gantry does not get exactly to its target position
        '''
        msg = Float32()
        msg.data = -value
        # rospy.loginfo(f"Gantry Z position set to {value}")
        self.gantry_z_publisher.publish(msg)

        # Wait for the Z-movement process to complete
        # target_Z = self.z_mapping[value]
        if value == 0:
            target_Z = -500
        target_Z = value*-1000
        while self.currentZ != int(target_Z): 
            rospy.loginfo(f"Gantry - Set Z: {value}, Current Z: {self.currentZ}, Target Z : {target_Z}")
            rospy.sleep(1)
        rospy.loginfo("Gantry Z-direction complete")
        
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

    def move_conveyor(self, distance_in_inches):
        """
        Moves the conveyor a specific distance in inches.

        :param distance_in_inches: Distance to move the conveyor in inches (negative for reverse).
        """
        if distance_in_inches == 0:
            rospy.loginfo("No movement required.")
            return

        direction = 1 if distance_in_inches > 0 else -1
        speed = direction * abs(self.conveyor_speed)

        # Calculate time required
        travel_time = abs(distance_in_inches) / abs(self.conveyor_speed)

        rospy.loginfo(f"Moving conveyor {distance_in_inches} inches at {speed} inches/sec for {travel_time:.2f} seconds.")

        # Start conveyor
        self.set_conveyor_main_speed(speed)

        # Wait for the travel time TODO: This may not be the best way to do this. Try multithreading.
        rospy.sleep(travel_time)

        # Stop conveyor
        self.set_conveyor_main_speed(0)
        rospy.loginfo("Conveyor movement complete.")

    def convert_cam2gantryPose(self, grasp_pose):
        # Define the transformation matrix (4x4)
        transform_matrix = np.array([
            [-1, 0, 0, self.cam2gantryHome_x],
            [0, -1, 0, self.cam2gantryHome_y],
            [0, 0, 1, self.cam2gantryHome_z],
            [0, 0, 0, 1]
        ])
        
        # Extract grasp pose coordinates and label
        x, y, z, label = grasp_pose
        
        # Convert grasp pose to homogeneous coordinates (4x1)
        pose_homogeneous = np.array([x, y, z, 1])
        
        # Apply the transformation matrix
        transformed_pose = np.dot(transform_matrix, pose_homogeneous)
        
        # Convert transformed coordinates from meters to inches
        transformed_pose_inches = transformed_pose[:3] * 39.3700787
        
        # Extract the new x, y, z values
        x_new, y_new, z_new = transformed_pose_inches
        
        # Cap x between 1 and 60, and z between 1 and 7
        x_new = np.clip(x_new, 1, 60)
        z_new = np.clip(z_new, 1, 7)
        
        # Return the transformed pose (x, y, z, label)
        return [x_new, y_new, z_new, label]
    
    def truncate_to_three_decimal_places(value): #TODO: move this to a helper function
        """Truncate a floating-point value to three decimal places."""
        return int(value * 1000) / 1000


    def receive_data(self, req):
        '''
        Main gantry function to work through the process of picking up an object and placing it in the correct location.
        '''
      # extrast grasp pose
        sucPose = req.grasps.data # [[x, y, z, label], .....]
      # convert grasp pose from camera frame to gantry frame
        gantryPose = self.convert_cam2gantryPose(sucPose[0])
      # truncate pose values to three decimal places
        gantryPose = [self.truncate_to_three_decimal_places(value) for value in gantryPose]
      # send gantry home (required on startup) NOTE: move this to startup
        rospy.loginfo("gantry_control - Gantry to Home")
        self.callbackHome()
      # use y value for conveyor control - send item from camera view to gantry alignment
        # TODO: implement conveyor control
        self.move_conveyor(gantryPose[1])
        # use x value of gantry alignment on conveyor
        rospy.loginfo(f"gantry_control - Gantry to X: {gantryPose[0]}")
        self.setXposition(gantryPose[0])
        # use z value for depth
        rospy.loginfo(f"gantry_control - Gantry to Z: {gantryPose[2]}")
        self.setZposition(gantryPose[2])
        # turn on suction
        rospy.loginfo("gantry_control - Pneumatic gripper On")
        self.pneumatic_gripper(True)
        # raise end effector up to z=1
        rospy.loginfo("gantry_control - Gantry to Z: 1")
        self.setZposition(1)
        # TODO: decide where to put material (left or right). Handle label here?
        if gantryPose[3] == 0:
            # move to the right
            rospy.loginfo("gantry_control - Right Chosen")
            self.setXposition(60)
        else:
            # move to the left
            rospy.loginfo("gantry_control - Left Chosen")
            self.setXposition(1)
        # turn off suction
        rospy.loginfo("gantry_control - Pneumatic gripper Off")
        self.pneumatic_gripper(False)
        rospy.loginfo("gantry_control - Done")

        # Generate response
        # response = SuctionSrvResponse()
        # response.grasps.data = flattened_suction_poses

        return True #TODO: return success or failure. Or, return the next action to take.

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # Spins until node is terminated        
        rospy.spin()


if __name__ == '__main__':
    GantryControl().run()