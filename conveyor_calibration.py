#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from robots_for_recycling.srv import conveyorCalibrationSrv, conveyorCalibrationSrvResponse
import pyrealsense2 as rs
import apriltag
import cv2
import time
import numpy as np
import yaml

'''
Update CMakeLists.txt

Ensure the CMakeLists.txt in your package includes the service definition:

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  conveyorCalibrationSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

---------------

Calling the Service from terminal:
$ rosservice call /conveyor_calibration "{speed_value: 10}"

---------------

Veryfy the Service:

$ rosservice list

Check for /conveyor_calibration in the output.

----------------

To get the service definition:

$ rosservice info /conveyor_calibration

----------------

To inspect the service type:

$ rosservice type /conveyor_calibration
Should return robots_for_recycling/conveyorCalibrationSrv

----------------

$ rossrv show robots_for_recycling/conveyorCalibrationSrv

Check the service definition.

----------------



'''


class ConveyorCalibration:
    def __init__(self):
        # ROS Publishers for conveyor control
        self.pub_main_conveyor = rospy.Publisher('/beltSpeed2', Int16, queue_size=1)
        rospy.Service('conveyor_calibration', conveyorCalibrationSrv, self.calibrate_conveyor)

        # RealSense pipeline setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        # AprilTag detector
        self.detector = apriltag.Detector()

        rospy.loginfo("Conveyor Calibration Service Ready.")

    def calibrate_conveyor(self, req):
        """Service callback to calibrate conveyor speeds."""
        speeds = [req.speed_value]  # Request speeds [-30, -20, ..., 10, 20]
        speed_mapping = {}

        for speed in speeds:
            rospy.loginfo(f"Setting conveyor speed to: {speed}")
            self.set_conveyor_speed(speed)

            # Track AprilTag movement
            real_speed = self.track_tag()
            speed_mapping[speed] = real_speed

            rospy.loginfo(f"Measured speed for {speed}: {real_speed:.3f} m/s")
        
        # Save calibration data
        self.save_calibration(speed_mapping)
        return conveyorCalibrationSrvResponse(True)

    def set_conveyor_speed(self, speed):
        """Set the speed of the main conveyor."""
        msg = Int16()
        msg.data = speed * 8.5  # Apply speed scaling
        self.pub_main_conveyor.publish(msg)
        time.sleep(2)  # Wait for speed to stabilize

    def track_tag(self):
        """Track AprilTag displacement over time and calculate real-world speed."""
        start_time = time.time()
        positions = []

        while time.time() - start_time < 5:  # Track for 5 seconds
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            tags = self.detector.detect(gray)

            if tags:
                tag = tags[0]
                positions.append(tag.center[0])  # Track X-axis center

            cv2.imshow("Tag Tracking", image)
            cv2.waitKey(1)

        # Calculate speed (pixels per second -> meters/second)
        if len(positions) > 1:
            pixel_displacement = positions[-1] - positions[0]
            real_distance = self.pixel_to_meter(pixel_displacement)
            duration = time.time() - start_time
            return real_distance / duration
        return 0

    def pixel_to_meter(self, pixel_displacement):
        """Convert pixel displacement to real-world meters."""
        PIXELS_PER_METER = 500  # Replace with calibrated value
        return pixel_displacement / PIXELS_PER_METER

    def save_calibration(self, data):
        """Save calibration results to a YAML file."""
        with open("conveyor_calibration.yaml", "w") as file:
            yaml.dump(data, file)
        rospy.loginfo("Calibration data saved to conveyor_calibration.yaml")

if __name__ == "__main__":
    rospy.init_node('conveyor_calibration_node')
    cal = ConveyorCalibration()
    rospy.spin()
