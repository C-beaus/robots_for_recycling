#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from robots_for_recycling.srv import CameraSrv, CameraSrvResponse
import pyrealsense2 as rs
from cv_bridge import CvBridge
import numpy as np
import cv2

class RealSenseService:
    def __init__(self):
        rospy.init_node('realsense_service')

        # Start up the camera to have it ready for frame requests.
        if not self.initialize_camera():
            rospy.logfatal("Failed to initialize camera. Shutting down node.")
            rospy.signal_shutdown("Camera initialization failed.")

        self.bridge = CvBridge()

        self.service = rospy.Service('camera_service', CameraSrv, self.capture_frames_callback)
        rospy.loginfo("RealSense service is ready to capture images.")

    def initialize_camera(self):
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            profile = self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
            
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            rospy.loginfo("Camera pipeline started successfully.")
            return True
        except Exception as e:
            rospy.logerr(f"Error while initializing camera: {e}")
            return False


    def capture_frames_callback(self, req):
        try:
            max_attempts = 10
            current_attempt = 0

            while current_attempt < max_attempts:
                # Wait for frames
                frames = self.pipeline.wait_for_frames()

                # Align depth frames to color frames
                aligned_frames = self.align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                capture_time = rospy.Time.now().secs

                if aligned_depth_frame and color_frame:

                    depth_image_np = np.asanyarray(aligned_depth_frame.get_data()) * self.depth_scale
                    color_image_np = np.asanyarray(color_frame.get_data())

                    # Convert to ROS Image messages
                    color_image = self.bridge.cv2_to_imgmsg(color_image_np, encoding="rgb8")
                    depth_image = self.bridge.cv2_to_imgmsg(depth_image_np, encoding="passthrough")

                    # Create and return the response
                    response = CameraSrvResponse()
                    response.rgb_image = color_image
                    response.depth_image = depth_image
                    response.timestamp.data = capture_time
                    rospy.loginfo(f"RGB and Depth pair collected at time stamp: {response.timestamp}")

                    return response

                current_attempt += 1
                rospy.logwarn(f"Attempt {current_attempt}/{max_attempts}: Valid frames not received. Retrying...")

            rospy.logerr(f"Failed to capture valid frames after {max_attempts} attempts.")
            return CameraSrvResponse()

        except Exception as e:
            rospy.logerr(f"Error while capturing frames: {e}")
            return CameraSrvResponse()
        

if __name__ == "__main__":
    try:
        service = RealSenseService()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    finally:
        if hasattr(service, 'pipeline'):
            service.pipeline.stop()
            rospy.loginfo("Camera pipeline stopped.")