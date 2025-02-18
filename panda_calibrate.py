import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import numpy as np
import cv2
from franka.panda_hw.src.panda_control_595 import PandaControl as PandaControlNode

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

            # print("panda_calibrate: waiting for 'q'")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return color_image

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return np.array(rvecs), np.array(tvecs), np.array(trash)

def detect_marker(image, visualize=True):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = gray.astype(np.uint8)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
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

        rvec, tvec, _ = my_estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
        
        cv2.drawFrameAxes(image_with_markers, camera_matrix, dist_coeffs, rvec, tvec, 0.1)  # Axis length 0.1m

    cv2.imshow('Detected ArUco Markers', image_with_markers)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return rvec, tvec


def main(T_ee_base=np.eye(4), T_marker_ee=np.eye(4)):

    robot_controller = PandaControlNode()
    pose = robot_controller.get_pose().pose # Could also be a problem, check if this is actually to EE and not the last joint before EE
    print(f'Current EE Pose: {pose}')
    positon = pose.position
    quat = pose.orientation

    z_offset = -6.8/100 # TODO double check offsets and their axes
    # x_offset = -2.2/100 # TODO double check offsets and their axes
    x_offset =  1.3/100 # TODO double check offsets and their axes
    
    # marker pose in end effector frame
    # T_marker_ee = np.array([[-1, 0, 0, x_offset],[0, 0, -1, 0],[0, 1, 0, z_offset],[0, 0, 0, 1]]) # TODO need to figure out the correct transformation
    # T_marker_ee = np.array([[-1, 0, 0, x_offset],[0, 0, 1, 0],[0, 1, 0, z_offset],[0, 0, 0, 1]]) # TODO need to figure out the correct transformation
    # T_marker_ee = np.array([[-.707, .707, 0, x_offset],[0, 0, -1, 0],[.707, .707, 0, z_offset],[0, 0, 0, 1]]) # TODO need to figure out the correct transformation
    T_marker_ee = np.array([[0, 0, 1, x_offset],[1, 0, 0, 0],[0, 1, 0, z_offset],[0, 0, 0, 1]]) # TODO need to figure out the correct transformation
    
    translation = np.array([positon.x, positon.y, positon.z])

    q = [quat.x, quat.y, quat.z, quat.w]
    rotation = R.from_quat(q)
    R_mat = rotation.as_matrix()

    T_ee_base[:3, :3] = R_mat
    T_ee_base[:3, 3] = translation


    marker_size= 0.050
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
        T_camera_marker[:3, 3] = tvec.ravel()
        c_m = np.linalg.inv(T_camera_marker)
        c_ee = np.dot(T_marker_ee, c_m)
        m_b = np.dot(T_ee_base, T_marker_ee)
        c_b_1 = np.dot(m_b, c_m)
        c_b_2 = np.dot(T_ee_base, c_ee)
        print(f'camera->marker:\n {c_m}')
        print(f'marker->ee:\n {T_marker_ee}')
        print(f'camera->ee:\n {c_ee}')
        print(f'ee->base:\n {T_ee_base}')
        print(f'marker->base:\n {m_b}')
        print(f'camera->base:\n {c_b_1}')
        print(f'camera->base:\n {c_b_2}')


        T_camera_base = np.dot(np.dot(T_ee_base, T_marker_ee), np.linalg.inv(T_camera_marker))
        
        print(f"Transformation (Camera to Marker): \n{T_camera_marker}")
        print(f"Extrinsic Transformation (Camera to Base): \n{np.array2string(T_camera_base, separator=',')}")

    else:
        return

main()