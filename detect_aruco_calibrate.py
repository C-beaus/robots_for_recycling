#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
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

# Approx. Transformation of camera (Intel RealSense D435i) XYZ [meters] w.r.t Base Frame XYZ [meters] of Franka Emika Panda.
# T_cam_to_base = np.array([
#     [ 0, 1,  0, 0.501],
#     [-1,  0,  0, 0.012+0.070],
#     [ 0,  0, -1, 0.733-0.100],
#     [ 0,  0,  0, 1.000]
# ]) # used with previous home: [-0.731, -0.432, 0.579, -1.884, 0.213, 1.518, 0.602] in panda_control_595.py, def set_def_pos(self).

pandaManipulator_x = 0.475 # [meters]
pandaManipulator_y = -0.008
pandaManipulator_z = 0.762
cam_wrt_pandaManipulator_x = 0.060
cam_wrt_pandaManipulator_y = 0
cam_wrt_pandaManipulator_z = -0.090
T_cam_to_base = np.array([
    [ 0, -1,  0, 0.535],
    [-1,  0,  0, -0.008],
    [ 0,  0, -1, 0.852],
    [ 0,  0,  0, 1.000]
]) # used with newer home: [0.284, -0.103, -0.266, -1.318, -0.043, 1.255, 0.788] inpanda_control_595.py, def set_def_pos(self).

def capture_color_image():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image
    finally:
        pipeline.stop()

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

    # Define joint positions
    home_joint = [0.284, -0.103, -0.266, -1.318, -0.043, 1.255, 0.788]
    left_joint = [0.420, 0.430, 0.517, -0.988, -0.167, 1.362, 1.603]
    right_joint = [-0.538, 0.487, -0.270, -0.899, 0.093, 1.352, -0.021]

    scan_poses = [("home", home_joint), ("left", left_joint), ("right", right_joint)]

    detected_markers = {}

    for label, joint in scan_poses:
        print(f"\n[INFO] Moving to {label} position...")
        robot.move_joint(joint)
        # rospy.sleep(2.0)

        image = capture_color_image()
        corners, ids = detect_markers(image)

        if ids is not None:
            rvecs, tvecs = estimate_pose(corners)
            ee_pose = robot.get_pose().pose

            base_R = R.from_quat([ee_pose.orientation.x, ee_pose.orientation.y,
                                  ee_pose.orientation.z, ee_pose.orientation.w]).as_matrix()
            base_T = np.eye(4)
            base_T[:3, :3] = base_R
            base_T[:3, 3] = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]

            for i, marker_id in enumerate(ids.flatten()):
                T_camera_marker = pose_to_matrix(rvecs[i], tvecs[i])
                marker_cam = np.linalg.inv(T_camera_marker)
                # marker_base = base_T @ robot.T @ marker_cam
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
    # import rospy
    # rospy.init_node("detect_aruco_calibrate")
    main()
