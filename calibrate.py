import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import numpy as np
import cv2

def align_depth_to_color():

    pipeline = rs.pipeline()

    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the pipeline
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow('Color Image', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return color_image


def detect_marker(image, visualize=True):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = gray.astype(np.uint8)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(gray)

    if len(corners) > 0:
        return corners, ids
    else:
        print('No markers detected.')
        return None, None

def estimate_marker_pose(corners, ids, image, marker_size, camera_matrix, dist_coeffs):

    image_with_markers = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
    
    for i in range(len(ids)):

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
        
        cv2.drawFrameAxes(image_with_markers, camera_matrix, dist_coeffs, rvec, tvec, 0.1)  # Axis length 0.1m

        print(f"Marker ID: {ids[i]}")
        print(f"Rotation Vector (rvec): {rvec}")
        print(f"Translation Vector (tvec): {tvec}")

    cv2.imshow('Detected ArUco Markers', image_with_markers)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return rvec, tvec


def main(T_ee_base=np.eye(4), T_marker_ee=np.eye(4)):

    offset = 1.9/100 # 1.9cm

    T_marker_ee = np.array([[0, 0, 1, 0],[1, 0, 0, 0],[0, 1, 0, -offset],[0, 0, 0, 1]])

    translation = np.array([0.4545, 0.0709, 0.5907])

    q = [0.7354, -0.0515, 0.673, 0.0596]
    rotation = R.from_quat(q)
    R_mat = rotation.as_matrix()

    T_ee_base[:3, :3] = R_mat
    T_ee_base[:3, 3] = translation

    marker_size= 0.025
    image = align_depth_to_color()

    ppx=321.1669921875
    ppy=231.57203674316406
    fx=605.622314453125
    fy=605.8401489257812

    camera_matrix = np.array([[fx, 0, ppx],
                            [0, fy, ppy],
                            [0, 0, 1]], dtype=np.float32)

    dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)
    corners, ids = detect_marker(image)
    if corners:
        rvec, tvec = estimate_marker_pose(corners, ids, image, marker_size, camera_matrix, dist_coeffs)
        R_camera_marker, _ = cv2.Rodrigues(rvec)
        T_camera_marker = np.eye(4)
        T_camera_marker[:3, :3] = R_camera_marker
        T_camera_marker[:3, 3] = tvec
        T_camera_base = np.dot(np.dot(T_ee_base, T_marker_ee), np.linalg.inv(T_camera_marker))
        
        print(f"Transformation (Camera to Marker): {T_camera_marker}")
        print(f"Extrinsic Transformation (Camera to Base): {np.array2string(T_camera_base, separator=',')}")

    else:
        return

main()
