import os
import sys
script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "antipodal_grasp_network")
if package_path not in sys.path:
    sys.path.append(package_path)
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import cv2
class Visualizer:
    def __init__(self):
        rospy.init_node("grasp_viz_node")
        self.image = None
        self.window_name = "Grasps"        
        # Create a cv2 window
        self.window = cv2.namedWindow(self.window_name)
        self.img_sub = rospy.Subscriber('/visualization_images', Float64MultiArray, self.image_callback)
        self.grasp_sub = rospy.Subscriber('/visualization_grasps', Float64MultiArray, self.grasp_callback)
        rospy.loginfo('Grasp visualization node ready.')
        

    def grasp_callback(self, msg):

        if self.image is not None:
            flat_grasps = np.array(msg.data)
            reshaped_grasps = flat_grasps.reshape(-1, 5)
            rospy.loginfo("Received Grasps for visualization")

            for row in reshaped_grasps:
                y, x, angle, width, length = row
                angle_degrees = angle * 180 / np.pi
                size = (width, length)  # Size of the rectangle (width, height)
                # Create a rotated rectangle
                rotated_rect = ((x, y), size, angle_degrees)
                
                # Get the four corners of the rotated rectangle
                box_points = cv2.boxPoints(rotated_rect)
                box_points = np.int0(box_points)  # Convert to integer points

                # Draw the rectangle on the image
                cv2.polylines(self.image, [box_points], isClosed=True, color=255, thickness=2)

            cv2.imshow(self.window_name, self.image)
            cv2.waitKey(1000)
        else:
            rospy.logwarn("No image for visualization.")

    def image_callback(self, msg):

        image = np.array(msg.data)
        self.image = image.reshape(480, 640)
        rospy.loginfo("image received")
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # Spins until node is terminated        
        rospy.spin()


        
if __name__ == '__main__':
    Visualizer().run()


