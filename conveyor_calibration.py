# #!/usr/bin/env python3
# import rospy
# import cv2
# import numpy as np
# from std_msgs.msg import Int16
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image

# class ConveyorCalibration:
#     def __init__(self):
#         rospy.init_node('conveyor_calibration', anonymous=True)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
#         self.image_pub = rospy.Publisher('/camera/rgb/image_modified', Image, queue_size=1)
#         self.conveyor_pub = rospy.Publisher('/beltSpeed2', Int16, queue_size=1)
#         self.circle_detected = False
#         self.circle_position = None
#         self.speeds = [2, 4, 6, 8, 10]
#         self.current_speed_index = 0
#         self.calibration_data = []

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")
#             return
#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (11, 11), 0)
#         circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1.2, 100, param1=50, param2=30, minRadius=20, maxRadius=50)

#         if circles is not None:
#             circles = np.round(circles[0, :]).astype('int')
#             for (x, y, r) in circles:
#                 if r > 45 and r < 55:  # Assuming the circle radius is around 50 pixels
#                     self.circle_detected = True
#                     self.circle_position = (x, y)
#                     cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
#                     cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
#                     cv2.putText(cv_image, "Circle Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#                     break
#         else:
#             self.circle_detected = False
#             cv2.putText(cv_image, "Circle Not Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

#         try:
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
#         except CvBridgeError as e:
#             rospy.logerr(f"CvBridge Error: {e}")

#     def move_conveyor(self, speed):
#         self.conveyor_pub.publish(speed)

#     def stop_conveyor(self):
#         self.conveyor_pub.publish(0)

#     def calibrate(self):
#         rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             if self.circle_detected:
#                 initial_position = self.circle_position[0]
#                 self.move_conveyor(self.speeds[self.current_speed_index])
#                 rospy.sleep(1)  # Allow some time for the conveyor to move

#                 while self.circle_position[0] < 640:  # Assuming the image width is 640 pixels
#                     rate.sleep()

#                 final_position = self.circle_position[0]
#                 self.stop_conveyor()
#                 rospy.sleep(1)  # Allow some time for the conveyor to stop

#                 distance = final_position - initial_position
#                 time_taken = 1.0  # Assuming it took 1 second to move the circle across the image
#                 velocity = distance / time_taken
#                 acceleration = velocity / time_taken

#                 self.calibration_data.append({
#                     'speed': self.speeds[self.current_speed_index],
#                     'distance': distance,
#                     'velocity': velocity,
#                     'acceleration': acceleration
#                 })

#                 self.current_speed_index += 1
#                 if self.current_speed_index >= len(self.speeds):
#                     break

#                 self.move_conveyor(-self.speeds[self.current_speed_index])
#                 rospy.sleep(1)  # Allow some time for the conveyor to move back

#                 while self.circle_position[0] > 0:
#                     rate.sleep()

#                 self.stop_conveyor()
#                 rospy.sleep(1)  # Allow some time for the conveyor to stop

#         print('Calibration Data:', self.calibration_data)

# if __name__ == '__main__':
#     try:
#         calibration = ConveyorCalibration()
#         calibration.calibrate()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         cv2.destroyAllWindows()