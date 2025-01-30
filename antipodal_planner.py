#!/usr/bin/env python3
import sys
import os
import rospkg
script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "antipodal_grasp_network")
if package_path not in sys.path:
    sys.path.append(package_path)
import rospy
from std_msgs.msg import Float64MultiArray
from run_grasp_generator_modified import run_inference
from inference.grasp_generator_modified import GraspGenerator
import numpy as np
from robots_for_recycling.srv import GraspSrv, rgbdSrv, GraspSrvResponse, rgbdSrvResponse
from cv_bridge import CvBridge
import cv2

"""
This class handles translating bounding boxes for use by the manipulation team
"""
class AntipodalPlanner:
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("Antipodal_Planner")
        self.boxes = None
        # self.cam2bot = np.eye(4) # Change this to actual extrinsics. Ensure it is being passed through till the end.
        self.cam2bot = np.matrix('0 1 0; -1 0 0; 0 0 1; 0 0 0 1') # new camera orientation?

        self.grasp_generation_service = rospy.Service('run_grasp_model', rgbdSrv, self.run_grasp_inference)
        self.grasp_selection_service = rospy.Service('select_grasps_from_bbs', GraspSrv, self.select_bbs_grasps)
        self.grasp_pub_for_viz = rospy.Publisher('/visualization_grasps', Float64MultiArray, queue_size=10)
        self.img_pub_for_viz = rospy.Publisher('/visualization_images', Float64MultiArray, queue_size=10)

        # Create subscriber
        rospack = rospkg.RosPack()
        current_dir = rospack.get_path("robots_for_recycling")

        print(current_dir)

        self.model_path = os.path.join(current_dir, 'antipodal_grasp_network/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch16/epoch_17_iou_0.96')
        self.generator = GraspGenerator(saved_model_path=self.model_path)
        self.generator.load_model()
        rospy.loginfo(f'Grasp Node Ready.')

        self.bridge = CvBridge()
        self.depth_img = None
        self.color_img = None
        self.q_img = None
        self.ang_img = None
        self.width_img = None


    def run_grasp_inference(self, req):
        self.depth_img = np.copy(self.bridge.imgmsg_to_cv2(req.depth_image, desired_encoding="passthrough"))
        self.color_img = np.copy(self.bridge.imgmsg_to_cv2(req.rgb_image, desired_encoding="rgb8"))
        self.q_img, self.ang_img, self.width_img = run_inference(generator=self.generator, color_image=self.color_img, depth_image=self.depth_img, use_cam=False)
        
        if self.q_img.size > 0 and self.ang_img.size > 0 and self.width_img.size > 0:
            response = rgbdSrvResponse()
            response.infer_success = True
            return response




    def generatePose(self):

        metric_grasp_poses, grasp_params_for_viz = self.generator.generate_poses(q_img=self.q_img, ang_img=self.ang_img, width_img=self.width_img, depth=self.depth_img, 
                                                bboxes=self.boxes, camera2robot=self.cam2bot, ppx=321.1669921875, ppy=231.57203674316406, 
                                                fx=605.622314453125, fy=605.8401489257812)
        metric_grasp_poses = np.array(metric_grasp_poses, dtype=np.float64)
        flattened_grasp_poses = metric_grasp_poses.flatten()

        rospy.loginfo(f'Grasp poses: {metric_grasp_poses}')

        img_gray = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2GRAY)
        grasp_msg = Float64MultiArray()
        img_msg = Float64MultiArray()
        img_msg.data = img_gray.flatten()
        grasp_msg.data = grasp_params_for_viz
        self.grasp_pub_for_viz.publish(grasp_msg)
        self.img_pub_for_viz.publish(img_msg)
        
        return flattened_grasp_poses

    def select_bbs_grasps(self, req):

        boxes_array = np.array(req.bbs.data, dtype=np.float64)
        self.boxes = boxes_array.reshape(-1, 5)
        rospy.loginfo(f'Received bounding boxes {self.boxes}')

        # Generate the pose with the bounding boxes
        flattened_grasp_poses = self.generatePose()
        response = GraspSrvResponse()
        response.grasps.data = flattened_grasp_poses
        
        return response


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # Spins until node is terminated        
        rospy.spin()


        
if __name__ == '__main__':
    AntipodalPlanner().run()