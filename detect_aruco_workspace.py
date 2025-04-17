import rospy
import cv2
import yaml
import numpy as np
from cv_bridge import CvBridge
from robots_for_recycling.srv import CameraSrv
from panda_control_595 import PandaControl  # Make sure it's importable

ARUCO_DICT = cv2.aruco.DICT_4X4_50
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)


LEFT_JOINTS = [0.420, 0.430, 0.517, -0.988, -0.167, 1.362, 1.603]
RIGHT_JOINTS = [-0.538, 0.487, -0.270, -0.899, 0.093, 1.352, -0.021]

SAVE_FILE = "aruco_marker_positions.yaml"

class ArucoScanner:
    def __init__(self, panda_controller):
        self.panda = panda_controller
        self.bridge = CvBridge()
        rospy.wait_for_service('camera_service')
        self.get_rgbd = rospy.ServiceProxy('camera_service', CameraSrv)

    def scan_at_pose(self, joint_pose):
        rospy.loginfo("Moving to pose...")
        self.panda.move_joint(joint_pose)
        rospy.sleep(1.0)

        response = self.get_rgbd()
        rgb_image = self.bridge.imgmsg_to_cv2(response.rgb_image, desired_encoding="rgb8")
        depth_image = self.bridge.imgmsg_to_cv2(response.depth_image, desired_encoding="passthrough")

        corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, aruco_dict, parameters=aruco_params)

        marker_positions = {}
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                center_px = np.mean(corners[i][0], axis=0)
                px, py = int(center_px[0]), int(center_px[1])
                depth = depth_image[py, px] / 1000.0  # mm to meters
                if np.isnan(depth) or depth == 0:
                    continue

                x = (px - 321.1669921875) * depth / 605.622314453125
                y = (py - 231.57203674316406) * depth / 605.8401489257812
                z = depth

                pos_camera = np.array([x, y, z, 1.0])
                pos_robot = self.panda.tf_cam_to_panda(pos_camera)[:3].tolist()

                marker_positions[int(marker_id)] = pos_robot
                rospy.loginfo(f"Marker {marker_id} → {pos_robot}")

        return marker_positions

    def run(self):
        rospy.loginfo("Starting ArUco scanning routine")
        all_markers = {}

        for pose in [LEFT_JOINTS, RIGHT_JOINTS]:
            detected = self.scan_at_pose(pose)
            all_markers.update(detected)

        with open(SAVE_FILE, 'w') as f:
            yaml.dump(all_markers, f)
        rospy.loginfo(f"Saved marker positions to {SAVE_FILE}")


if __name__ == '__main__':
    rospy.init_node('aruco_scanner_node')
    panda = PandaControl()
    scanner = ArucoScanner(panda)
    scanner.run()
