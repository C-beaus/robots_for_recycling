#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import argparse
from franka.panda_hw.src.panda_control_595 import PandaControl as PandaControlNode

# Camera intrinsics
ppx, ppy = 321.1669921875, 231.57203674316406
fx, fy = 605.622314453125, 605.8401489257812
camera_matrix = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

marker_size = 0.05  # 50 mm

T_cam_to_base = np.array([
    [ 0, -1,  0, 0.501],
    [-1,  0,  0, 0.082],
    [ 0,  0, -1, 0.633],
    [ 0,  0,  0, 1.000]
])

def detect_markers(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    return detector.detectMarkers(gray)

def estimate_pose_and_draw(image, corners, ids):
    marker_points = np.array([
        [-marker_size/2, marker_size/2, 0],
        [ marker_size/2, marker_size/2, 0],
        [ marker_size/2,-marker_size/2, 0],
        [-marker_size/2,-marker_size/2, 0]
    ], dtype=np.float32)

    for i in range(len(ids)):
        _, rvec, tvec = cv2.solvePnP(marker_points, corners[i], camera_matrix, dist_coeffs)
        cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
    image_with_boxes = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
    return image_with_boxes

def run_live_view():
    print("[INFO] Starting live camera feed. Press 'q' to capture and display markers.")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow("Live RealSense Feed", color_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    # corners, ids = detect_markers(color_image)
    corners, ids, _ = detect_markers(color_image)

    if ids is not None:
        print(f"[INFO] Detected markers: {ids.flatten()}")
        image_with_boxes = estimate_pose_and_draw(color_image.copy(), corners, ids)
        cv2.imshow("Detected Markers", image_with_boxes)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("[INFO] No markers detected in captured frame.")

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

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        pipeline.stop()

        corners, ids = detect_markers(image)


        if ids is not None:
            rvecs, tvecs = [], []
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
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--live", action="store_true", help="Enable real-time camera viewer")
    args = parser.parse_args()

    if args.live:
        run_live_view()
    else:
        main()
