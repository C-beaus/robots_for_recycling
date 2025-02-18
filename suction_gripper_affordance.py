
import numpy as np
from scipy.ndimage import convolve
import open3d as o3d
import cv2
import matplotlib.pyplot as plt

class SuctionGenerator:
    
    def __init__(self, cam2robot, grid_resoltuion, coverage_threshold, belt_depth):
        
        self.cam2robot = cam2robot
        self.grid_resolution = grid_resoltuion
        self.coverage_threshold = coverage_threshold
        self.belt_depth = belt_depth # this is in the camera frame
        self.belt_points_margin = 0.005
        self.thin_thresold = 0.05 # Consider object thin if surface is only 0.05 cm above belt surface
    
    def remove_belt_points(self, depth):

        if abs(np.min(depth) - self.belt_depth) <= self.thin_thresold: # object will get removed because it is so thin, so don't do anything to depth map
            return depth
        else:
            # considering z-axis of camera is pointing down
            depth = np.where(depth <= self.belt_depth - self.belt_points_margin, depth, 0)
            return depth
 


    def generate_occupancy_grid(self, flat_3D_points):

        # Calculate the number of grid cells required
        min_x, min_y = np.min(flat_3D_points[:, :2], axis=0)
        max_x, max_y = np.max(flat_3D_points[:, :2], axis=0)
        
        # Determine grid width and height
        grid_width = int(np.ceil((max_x - min_x) / self.grid_resolution))
        grid_height = int(np.ceil((max_y - min_y) / self.grid_resolution))
        

        # Create empty occupancy grid
        occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.float32)

        # Map the 3D points to grid cells and update the occupancy grid
        grid_x_indices = ((flat_3D_points[:, 0] - min_x) / self.grid_resolution).astype(int)
        grid_y_indices = ((flat_3D_points[:, 1] - min_y) / self.grid_resolution).astype(int)
        # Ensure indices are within bounds
        valid_indices = (grid_x_indices >= 0) & (grid_x_indices < grid_width) & \
                        (grid_y_indices >= 0) & (grid_y_indices < grid_height)

        # Set corresponding grid cells to 1 (occupied)
        occupancy_grid[grid_y_indices[valid_indices], grid_x_indices[valid_indices]] = 1

        return occupancy_grid, min_x, min_y



    def create_suction_kernel(self, triangle_vertices, suction_radius, save=False):

        # Convert the suction radius in grid units
        suction_radius_grid = suction_radius / self.grid_resolution
        
        # Calculate the bounding box of the triangle vertices (in metric units)
        min_x, min_y = np.min(triangle_vertices[:, :2], axis=0)
        max_x, max_y = np.max(triangle_vertices[:, :2], axis=0)
        
        # Extend the bounding box to cover the suction radius (in metric units)
        min_x -= suction_radius
        min_y -= suction_radius
        max_x += suction_radius
        max_y += suction_radius
        
        # Convert the bounding box to grid units
        kernel_width = int(np.ceil((max_x - min_x) / self.grid_resolution))
        kernel_height = int(np.ceil((max_y - min_y) / self.grid_resolution))

        # Create an empty kernel
        kernel = np.zeros((kernel_height, kernel_width), dtype=np.float32)

        # Convert triangle vertices from metric coordinates to grid indices
        triangle_x_indices = ((triangle_vertices[:, 0] - min_x) / self.grid_resolution).astype(int)
        triangle_y_indices = ((triangle_vertices[:, 1] - min_y) / self.grid_resolution).astype(int)
        
        # Mark grid cells within the suction radius of each triangle vertex
        for (x_idx, y_idx) in zip(triangle_x_indices, triangle_y_indices):
            for i in range(kernel_height):
                for j in range(kernel_width):
                    # Calculate the distance from the grid cell to the triangle vertex
                    distance = np.sqrt((i - y_idx) ** 2 + (j - x_idx) ** 2)
                    if distance <= suction_radius_grid:
                        kernel[i, j] = 1

        kernel /= np.sum(kernel)
        kernel = np.rot90(kernel, 3)
        # Save kernel to reuse
        if save:
            np.save('kernel.npy', kernel)


        return kernel


    def choose_approach_point(self, occupancy_grid, suction_kernel, grid_min_X, grid_min_Y):

        # Check if the Region of Interest is atleast equal to or bigger than the suciton cup
        if occupancy_grid.shape[0] < suction_kernel.shape[0] or occupancy_grid.shape[1] < suction_kernel.shape[1]:
            print("Suction cup cannot be used for the given object. Object is too small.")
            return None

        # Generate affordance map
        suction_affordance = convolve(occupancy_grid, suction_kernel, mode='constant', cval=0)

        # self.plot_grid(suction_affordance)

        # Find indices of all points meeting the coverage threshold
        valid_indices = np.argwhere(suction_affordance > self.coverage_threshold)

        if valid_indices.size == 0:
            print("Suction cup cannot be used for the given object. No points meet the coverage threshold.")
            return None

        grid_center = np.array(occupancy_grid.shape) / 2

        distances = np.linalg.norm(valid_indices - grid_center, axis=1)

        # Select the index of the point closest to the center
        closest_index = valid_indices[np.argmin(distances)]

        X_real_world, Y_real_world = self.grid_to_world_coordinates(
            closest_index[1], closest_index[0], grid_min_X, grid_min_Y
        )

        print(f"Suction gripper point ({X_real_world}, {Y_real_world}) found.")

        return X_real_world, Y_real_world

        
    def grid_to_world_coordinates(self, grid_x, grid_y, grid_min_X, grid_min_Y):

        x = grid_min_X + (grid_x * self.grid_resolution)
        y = grid_min_Y + (grid_y * self.grid_resolution)

        return x, y
    
    def compute_z(self, point_xy, pcd):

        points = np.asarray(pcd.points)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        distances = np.sqrt((x - point_xy[0])**2 + (y - point_xy[1])**2)
        
        closest_index = np.argmin(distances)
        
        return z[closest_index]

    def plot_grid(self, grid):

        plt.imshow(grid, cmap='gray', interpolation='nearest')

        plt.colorbar(label='Grid')
        plt.xlabel('X')
        plt.ylabel('Y')

        plt.show()

    def plot_affordance(self, grid, max_point=None):

        plt.imshow(grid, cmap='viridis', interpolation='nearest')
        plt.colorbar(label='Suction Affordance')

        if max_point:
            plt.scatter(max_point[1], max_point[0], color='red', marker='x', s=100, label='Max Coverage Point')
            plt.annotate(f'Max Point {max_point}', (max_point[1] + 0.5, max_point[0] + 0.5), color='white', fontsize=12)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Suction Affordance Map')
        plt.legend()
        plt.show()

    def generate_suction(self, depth, bboxes, z_component_threshold):
        
        depth = depth.astype(np.float32)
        # Filter depth map to remove belt points
        normal_depth = cv2.normalize(depth, 0, 255)
        points = []
        cv2.imshow('window',normal_depth)
        cv2.waitKey(0)
        print(depth.shape)

        for bbox in bboxes:

            padded_crop = np.zeros(depth.shape)
            padded_crop = padded_crop.astype(np.float32)

            label, x_center, y_center, width, height = bbox
            bb_top_left_row = int(y_center - height/2)
            bb_bottom_right_row = int(y_center + height/2)

            bb_top_left_col = int(x_center - width/2)
            bb_bottom_right_col = int(x_center + width/2)

            cv2.rectangle(normal_depth, (bb_top_left_col, bb_top_left_row), (bb_bottom_right_col, bb_bottom_right_row), (0, 255, 0), 2)
            cv2.imshow('window', normal_depth)
            cv2.waitKey(0)

            depth_crop = depth[bb_top_left_row : bb_bottom_right_row, bb_top_left_col : bb_bottom_right_col]
            padded_crop[bb_top_left_row : bb_bottom_right_row, bb_top_left_col : bb_bottom_right_col] = depth_crop
            padded_crop = self.remove_belt_points(padded_crop)
            cv2.imshow('window', depth_crop)
            cv2.waitKey(0)
            print(depth_crop)

            # # # calculate principal point with respect to crop
            # crop_cx = 321.1669921875 - bb_top_left_col
            # crop_cy = 231.57203674316406 - bb_top_left_row

            intrinsics = o3d.camera.PinholeCameraIntrinsic()
            intrinsics.set_intrinsics(width=padded_crop.shape[1], height=padded_crop.shape[0], fx=605.622314453125, fy=605.8401489257812, cx=321.1669921875, cy=231.57203674316406)
            
            bbox_pcd = o3d.geometry.PointCloud.create_from_depth_image(
                                depth=o3d.geometry.Image(padded_crop),
                                intrinsic=intrinsics,
                                depth_scale=1.0,
                                depth_trunc=1.5,  # Maximum depth to consider
                                stride=1  # Use every pixel (adjust for downsampling)
                            )
            
            # o3d.visualization.draw_geometries([bbox_pcd])
            
            bbox_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                                        radius=0.1,
                                        max_nn=30
                                        ))
            
            normals = np.asarray(bbox_pcd.normals)
            flat_indices = np.where(np.abs(normals[:, 2]) >= z_component_threshold)[0]
            
            flat_pcd = bbox_pcd.select_by_index(flat_indices)
            # o3d.visualization.draw_geometries([flat_pcd])

            triangle_vertices = np.array([[2.5, 4], [2.5, -4], [-4, 0]])/100
            suction_radius = 1.25/100  # in meters

            occupancy_grid, min_x, min_y = self.generate_occupancy_grid(np.asarray(flat_pcd.points))
            # self.plot_grid(occupancy_grid)
            occupancy_grid = cv2.dilate(occupancy_grid, np.ones((5, 5), np.uint8), iterations=1)
            # self.plot_grid(occupancy_grid)
            # occupancy_grid = cv2.erode(occupancy_grid, np.ones((3, 3), np.uint8), iterations=1)
            # self.plot_grid(occupancy_grid)
            kernel = self.create_suction_kernel(triangle_vertices, suction_radius, save=False)
            # self.plot_grid(kernel)

            # Choose a valid point closest to bbox center
            point_xy = self.choose_approach_point(occupancy_grid, kernel, min_x, min_y)

            if point_xy:

                z = self.compute_z(point_xy, flat_pcd)

                point = np.array([point_xy[0], point_xy[1], z, 1])
                transformed_point = self.cam2robot @ point
                grasp_point = np.array([transformed_point[0], transformed_point[1], transformed_point[2], label])
                points.append(grasp_point)

                sphere_radius = 0.005 # meters
                mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
                mesh_sphere.paint_uniform_color([0.5, 0, 0.5])
                mesh_sphere.translate([grasp_point[0], grasp_point[1], grasp_point[2]])
    
                temp_pcd = o3d.geometry.PointCloud()
                temp_pcd.points = o3d.utility.Vector3dVector([grasp_point[:3]])

                temp_pcd.colors = o3d.utility.Vector3dVector([[1, 0, 0]])

                o3d.visualization.draw_geometries([flat_pcd, temp_pcd, mesh_sphere])
        
        return np.array(points)
