#!/usr/bin/env python3

import cv2
import numpy as np
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
from pathlib import Path
from franka.panda_hw.src.panda_control_595 import PandaControl as PandaControlNode

# Define known camera intrinsics for D435i
ppx = 321.1669921875
ppy = 231.57203674316406
fx = 605.622314453125
fy = 605.8401489257812
camera_matrix = np.array([[fx, 0, ppx],
                          [0, fy, ppy],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

marker_size = 0.05  # 50 mm

# save_dir = Path("/mnt/data/aruco_detections_2025-04-18_05-26-32")
# save_dir.mkdir(parents=True, exist_ok=True)

def draw_markers_with_axes(image, corners, ids):
    image_with_markers = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
    for i in range(len(ids)):
        marker_points = np.array([
            [-marker_size/2, marker_size/2, 0],
            [ marker_size/2, marker_size/2, 0],
            [ marker_size/2,-marker_size/2, 0],
            [-marker_size/2,-marker_size/2, 0]
        ], dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(marker_points, corners[i][0], camera_matrix, dist_coeffs)
        cv2.drawFrameAxes(image_with_markers, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
    return image_with_markers

def capture_color_image():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow('Live Camera Feed', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    return color_image

def detect_markers(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    return corners, ids

def estimate_pose(corners):
    rvecs, tvecs, _ = [], [], []
    marker_points = np.array([
        [-marker_size/2, marker_size/2, 0],
        [ marker_size/2, marker_size/2, 0],
        [ marker_size/2,-marker_size/2, 0],
        [-marker_size/2,-marker_size/2, 0]
    ], dtype=np.float32)
    for c in corners:
        _, rvec, tvec = cv2.solvePnP(marker_points, c, camera_matrix, dist_coeffs)
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs

def pose_to_matrix(rvec, tvec):
    R_mat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = tvec.flatten()
    return T

def main():
    robot = PandaControlNode()
    home_joint = [0.284, -0.103, -0.266, -1.318, -0.043, 1.255, 0.788]
    left_joint = [0.420, 0.430, 0.517, -0.988, -0.167, 1.362, 1.603]
    right_joint = [-0.538, 0.487, -0.270, -0.899, 0.093, 1.352, -0.021]
    scan_poses = [("home", home_joint), ("left", left_joint), ("right", right_joint)]

    detected_markers = {}

    for label, joint in scan_poses:
        print(f"\n[INFO] Moving to {label} position...")
        robot.move_joint(joint)

        image = capture_color_image()
        corners, ids = detect_markers(image)

        if ids is not None:
            vis_image = draw_markers_with_axes(image, corners, ids)

            # timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            # save_path = save_dir / f"{label}_{timestamp}.png"

            cv2.imshow("ArUco Markers with Axes", vis_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            rvecs, tvecs = estimate_pose(corners)
            ee_pose = robot.get_pose().pose
            T_ee_to_base = np.eye(4)
            T_ee_to_base[:3, :3] = R.from_quat([
                ee_pose.orientation.x,
                ee_pose.orientation.y,
                ee_pose.orientation.z,
                ee_pose.orientation.w
            ]).as_matrix()
            T_ee_to_base[:3, 3] = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]

            R_cam_to_ee = np.array([
                [ 0, -1, 0],
                [ 1,  0, 0],
                [ 0,  0, 1]
            ])
            T_cam_to_ee = np.eye(4)
            T_cam_to_ee[:3, :3] = R_cam_to_ee
            T_cam_to_ee[:3, 3] = [0.060, 0.000, 0.000]  # EE to camera

            T_cam_to_base = T_ee_to_base @ T_cam_to_ee

            for i, marker_id in enumerate(ids.flatten()):
                T_camera_marker = pose_to_matrix(rvecs[i], tvecs[i])
                marker_cam = np.linalg.inv(T_camera_marker)
                marker_base = T_cam_to_base @ marker_cam
                detected_markers[marker_id] = marker_base[:3, 3]
                print(f"[MARKER {marker_id}] Position in base frame: {marker_base[:3, 3]}")
        else:
            print(f"[INFO] No markers detected at {label} position.")

    print("\n=== Marker Positions Summary ===")
    for marker_id, position in detected_markers.items():
        print(f"ID {marker_id}: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")

    print("[INFO] Returning to home...")
    robot.move_joint(home_joint)

if __name__ == '__main__':
    main()
