from inference.grasp_generator_modified import GraspGenerator
import pyrealsense2 as rs
import numpy as np
import cv2

def capture_frames():
    
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and enable the bag file
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start the pipeline
    profile = pipeline.start(config)

    # Create an align object to align color frames to depth frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    # # Get intrinsic parameters for depth and color streams
    # color_stream = pipeline.get_active_profile().get_stream(rs.stream.color)
    # color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    i = 0
    try:
        while True:

            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.first(rs.stream.color)
            aligned_depth_frame = aligned_frames.get_depth_frame()

            # Keep looping until valid depth and color frames are received.
            if not color_frame and aligned_depth_frame:
                continue
            if i == 10:
                depth_image = np.asarray(aligned_depth_frame.get_data(), dtype=np.float32)
                color_image = np.asanyarray(color_frame.get_data())
                break
            i+=1
            
    
    except RuntimeError as e:
        print(f"Error occurred: {e}")
    finally:
        pipeline.stop()
        return color_image, depth_image, depth_scale
    
def process_frames(color_image, depth_image, depth_scale):

    depth_image *= depth_scale
    color_image = 255 - color_image
    grey_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    _, binary_mask = cv2.threshold(grey_img, 150, 250, cv2.THRESH_BINARY)
    largest_contour = find_largest_contour(binary_mask)
    color_mask = np.zeros_like(color_image)
    depth_mask = np.zeros_like(depth_image)
    cv2.drawContours(color_mask, [largest_contour], -1, (255, 255, 255), thickness=cv2.FILLED)
    color_image = cv2.bitwise_and(color_image, color_mask)
    depth_image = cv2.bitwise_and(depth_image, depth_mask)
    depth_image = np.expand_dims(depth_image, axis=2)

    return color_image, depth_image



def find_largest_contour(image):

    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None
    largest_contour = max(contours, key=cv2.contourArea)

    return largest_contour


def generate_poses(generator=None, bboxes=None, camera2robot=None, color_image=None, depth_image=None, use_cam=True):

    # color_image and depth_image obtained from either classification team or collected ourselves
    # Collect own images if classification team did not provide them
    if use_cam:
        color_image, depth_image, depth_scale = capture_frames()
    
    color_image, depth_image =  process_frames(color_image, depth_image, depth_scale)
    
    # grasp poses is a list of arrays like [np.array([x,y,z,angle, label]), ....]
    # label of object included with grasp
    grasp_poses = generator.generate(depth=depth_image, rgb=color_image, bboxes=bboxes, camera2robot=camera2robot, 
                                             ppx=321.1669921875, ppy=231.57203674316406, fx=605.622314453125, 
                                             fy=605.8401489257812)
    
    return np.array(grasp_poses, dtype=np.float64)