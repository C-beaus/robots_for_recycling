#!/usr/bin/env python3
# infer.py
import sys
import os
import logging
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision import transforms
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pyrealsense2 as rs
from cv_bridge import CvBridge

from torch.utils.data import DataLoader, Dataset
from torchvision import transforms

import cv2
from PIL import Image
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray, Float64
from robots_for_recycling.srv import ClassifySrv, ClassifySrvResponse

class ClassifyServer:
    def __init__(self):
        rospy.init_node("classify_server")
        rospy.sleep(1.0)
        rospy.loginfo("Classify Server Ready")

        self.s = rospy.Service('classify_waste', ClassifySrv, self.run_classifier)


        # Set up logging
        self.logger = logging.getLogger('infer_logger')
        self.logger.setLevel(logging.DEBUG)

        # Create handlers for console and file logging
        c_handler = logging.StreamHandler(sys.stdout)
        f_handler = logging.FileHandler('infer.log')

        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)

        # Create formatters and add them to handlers
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(formatter)
        f_handler.setFormatter(formatter)

        # Add handlers to the logger
        self.logger.addHandler(c_handler)
        self.logger.addHandler(f_handler)

        # Constants
        # CLASSES = ['__background__', 'recycling', 'nonrecycling']  # Include background
        # CLASSES = ['__background__', 'nonplastic', 'plastic']
        self.CLASSES = ['__background__', 'cardboard', 'glass', 'metal', 'paper', 'plastic']

        self.NUM_CLASSES = len(self.CLASSES)

        # if issues: make sure you have the full folder open so that relative filepaths work 
        current_dir = os.getcwd()
        print(current_dir)
        current_dir = os.path.join(current_dir, "src/robots_for_recycling")
        self.model_name = os.path.join(current_dir, 'models/mobilenet_ss_18_wd_0001_class_dataset/fasterrcnn_model.pth')
        self.confidence_threshold = 0.7

        self.bridge = CvBridge()


    def get_model_instance_segmentation(self, num_classes):
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=False)
        self.logger.info("Initialized Faster R-CNN model with MobileNetV3 backbone.")

        # Get the number of input features for the classifier
        in_features = model.roi_heads.box_predictor.cls_score.in_features

        # Replace the pre-trained head with a new one
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
        self.logger.info("Modified the model's box predictor to accommodate the new number of classes.")

        return model

    def infer_frame(self, model, frame, device):
        transform = transforms.ToTensor()
        input_tensor = transform(frame).to(device)

        with torch.no_grad():
            outputs = model([input_tensor])[0]
        return outputs

    def draw_bounding_boxes(self, frame, boxes, labels, scores, threshold=0.5): 
        for box, label, score in zip(boxes, labels, scores):
            if score > threshold:
                xmin, ymin, xmax, ymax = map(int, box)
                class_name = self.CLASSES[label]
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(frame, f'{class_name}: {score:.2f}', (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame

    def run_classifier(self, req):
        
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        model = self.get_model_instance_segmentation(self.NUM_CLASSES)
        # model.load_state_dict(torch.load('fasterrcnn_model.pth', map_location=device))
        model.load_state_dict(torch.load(self.model_name, map_location=device))

        model.eval().to(device)

        # # Open a connection to the webcam
        # cv2.VideoCapture()
        # for i in range(5):  # Testing indices from 0 to 4
        #     cap = cv2.VideoCapture(i)
        #     if cap.isOpened():
        #         print(f"Camera found at index {i}")
        #         # break
        #     cap.release()
        # else:
        #     print("No camera found")

        # # cap = cv2.VideoCapture(1)
        # cap = cv2.VideoCapture(4)
        
        # if not cap.isOpened():
        #     self.logger.error("Failed to open webcam.")
        #     sys.exit(1)

        # self.logger.info("Starting real-time object detection. Press 'q' to quit.")

        # # while True:
        # ret, frame = cap.read()
        # if not ret:
        #     self.logger.error("Failed to grab frame.")
        #     sys.exit(1)
            # break

        ros_img_msg = req.rgb_image
        rgb_frame = self.bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding="rgb8")

        # Convert frame to RGB and PIL Image
        # rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb_frame)

        # Run inference
        outputs = self.infer_frame(model, pil_image, device)
        
        # Get the boxes, labels, and scores
        boxes = outputs['boxes'].cpu().numpy()
        labels = outputs['labels'].cpu().numpy()
        scores = outputs['scores'].cpu().numpy()

        # Draw the bounding boxes and labels on the frame
        # frame = self.draw_bounding_boxes(frame, boxes, labels, scores, threshold=self.confidence_threshold)

        yolo_v5_array = []
        i = 0
        for box, label, score in zip(boxes, labels, scores):
            if score > self.confidence_threshold:
                xmin, ymin, xmax, ymax = map(int, box)
                width = xmax - xmin
                height = ymax - ymin
                center_x = (xmax + xmin) / 2
                center_y = (ymax + ymin) / 2
                yolo_v5_array.append([label, center_x, center_y, width, height])
                i += 1
        yolo_v5_array = np.array(yolo_v5_array, dtype=np.float64).flatten()

        response = ClassifySrvResponse()
        response.bbs = yolo_v5_array
        rospy.loginfo(f'Sending bounding boxes {response.bbs}')

        self.logger.info("Real-time detection ended.")

        return response

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    
    ClassifyServer().run()
