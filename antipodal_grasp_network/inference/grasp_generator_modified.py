import os
import time

import matplotlib.pyplot as plt
import numpy as np
import torch
import cv2
from scipy.ndimage import distance_transform_edt
from hardware.camera import RealSenseCamera
from hardware.device import get_device
from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import Grasp, detect_grasps
from utils.visualisation.plot import plot_grasp
from skimage.feature import peak_local_max


class GraspGenerator:
    def __init__(self, saved_model_path):

        self.saved_model_path = saved_model_path

        self.saved_model_path = saved_model_path
        self.model = None
        self.device = None

        # self.cam_pose = np.loadtxt('saved_data/camera_pose.txt', delimiter=' ')
        self.cam_pose = np.eye(4,4)

        self.cam_data = CameraData(include_depth=True, include_rgb=True)

        homedir = os.path.join(os.path.expanduser('~'), "grasp-comms")
        self.grasp_request = os.path.join(homedir, "grasp_request.npy")
        self.grasp_available = os.path.join(homedir, "grasp_available.npy")
        self.grasp_pose = os.path.join(homedir, "grasp_pose.npy")


    def load_model(self):
        print('Loading model... ')
        self.model = torch.load(self.saved_model_path, map_location=torch.device('cpu'))
        # Get the compute device
        self.device = get_device(force_cpu=True)
    

    def detect_grasps_bboxes(self, bboxes, q_img, ang_img, width_img):
        
        grasps = []
        labels = []

        for bbox in bboxes:

            label, x_center, y_center, width, height = bbox
            x_top_left = int(x_center - width / 2)
            y_top_left = int(y_center - height / 2)
            x_bottom_right = int(x_center + width / 2)
            y_bottom_right = int(y_center + height / 2)
            roi = q_img[y_top_left:y_bottom_right, x_top_left:x_bottom_right]
            
            best_grasp = list(np.unravel_index(np.argmax(roi), roi.shape))

            best_grasp[0] = best_grasp[0] + y_top_left
            best_grasp[1] = best_grasp[1] + x_top_left

            best_grasp = tuple(best_grasp)
            best_grasp_angle = ang_img[best_grasp]
            g = Grasp(best_grasp, best_grasp_angle)

            if width_img is not None:
                g.length = width_img[best_grasp]
                g.width = g.length / 2
                grasps.append(g)
                labels.append(label)

        return grasps, labels
            
    def infer_from_model(self, depth, rgb, filter_q_img=False):
        
        self.filter_q_img = filter_q_img

        x, _, _ = self.cam_data.get_data(rgb=rgb, depth=depth)

        # Predict the grasp pose using the saved model
        with torch.no_grad():
            xc = x.to(self.device)
            pred = self.model.predict(xc)

        q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])

        if self.filter_q_img:
            q_img = np.where(np.squeeze(depth, axis=2)==0, 0, q_img) # only keep quality score for pixels for which depth is available

        return q_img, ang_img, width_img
    
    def find_nearest_valid_depth(self, depth_img, target_point):

        valid_mask = depth_img > 0
        _ , indices = distance_transform_edt(~valid_mask, return_indices=True)
        nearest_row, nearest_col = indices[:, target_point[0], target_point[1]]
        depth = depth_img[nearest_row, nearest_col]
        return depth

    def generate_poses(self, q_img, ang_img, width_img, depth, bboxes, camera2robot=np.eye(4), ppx=321.1669921875, ppy=231.57203674316406, 
                 fx=605.622314453125, fy=605.8401489257812): # Currently runs inference on entire image instead of each individual bounnding boxes

        grasps, labels = self.detect_grasps_bboxes(bboxes, q_img, ang_img, width_img)

        grasp_object_params = []
        metric_grasp_poses = []

        for i in range(len(grasps)):

            grasp_depth = depth[grasps[i].center[0], grasps[i].center[1]]
            # Get grasp position from model output
            if self.filter_q_img or grasp_depth != 0: # check if valid depth at pixel exists (valid depth will 
                                                        #always exist if you filter q_img based on valid depths
                                                        #already)
                pos_z = grasp_depth
            else:   # otherwise assigned nearest valid depth
                pos_z = self.find_nearest_valid_depth(depth, grasps[i].center)

            pos_x = np.multiply(grasps[i].center[1] - ppx,
                                pos_z / fx)
            pos_y = np.multiply(grasps[i].center[0] - ppy,
                                pos_z / fy)
            # TESTING UPDATE WIDTH
            grasps[i].width = np.multiply(grasps[i].width, pos_z / fx)

            grasp_object_params.append([grasps[i].center[0], grasps[i].center[1], grasps[i].angle, grasps[i].width, grasps[i].length])

            if pos_z == 0:
                print("Invalid Depth Found")
                continue

            target = np.asarray([pos_x, pos_y, pos_z])
            target.shape = (3, 1)

            target_position = np.dot(camera2robot[0:3, 0:3], target) + camera2robot[0:3, 3:]
            target_position = target_position[0:3, 0]

            # Convert camera to robot angle
            angle = np.asarray([0, 0, grasps[i].angle])
            angle.shape = (3, 1)
            target_angle = np.dot(camera2robot[0:3, 0:3], angle)

            # Concatenate grasp pose with grasp angle
            grasp_pose = np.append(target_position, target_angle[2])
            grasp_pose = np.append(grasp_pose, grasps[i].width)
            grasp_pose = np.append(grasp_pose, labels[i])
            metric_grasp_poses.append(grasp_pose)
            flat_grasp_obj_params = np.array(grasp_object_params).flatten()
        return metric_grasp_poses, flat_grasp_obj_params