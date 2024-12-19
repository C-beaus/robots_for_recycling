import os
import sys
script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "antipodal_grasp_network")
if package_path not in sys.path:
    sys.path.append(package_path)
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from utils.dataset_processing.grasp import Grasp
from utils.visualisation.plot import plot_grasp
import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self):

        self.image = None
        self.img_sub = rospy.Subscriber('/visualization_images', Float64MultiArray, self.image_callback)
        self.grasp_sub = rospy.Subscriber('/visualization_grasps', Float64MultiArray, self.grasp_callback)
        self.fig = plt.figure(figsize=(10, 10))
        

    def grasp_callback(self, msg):

        if self.image:
            flat_grasps = np.array(msg.data)
            reshaped_grasps = flat_grasps.reshape(-1, 4)
            rospy.loginfo("Received Grasps for visualization")

            grasps = []

            for center, angle, width, length in reshaped_grasps:

                g = Grasp(center, angle)
                g.length = length
                g.width = width
                grasps.append(g)

            plot_grasp(fig=self.fig, rgb_img=self.image, grasps=grasps, save=False)
        else:
            rospy.logwarn("No image for visualization.")

    def image_callback(self, msg):

        image = np.array(msg.data)
        self.image = image.reshape(480, 640)



