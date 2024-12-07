import numpy as np
from scipy.ndimage import convolve
import open3d as o3d
import cv2
import matplotlib.pyplot as plt

class SuctionGenerator:
    def __init__(self):
        pass
    def generate_occupancy_grid(self, flat_3D_points, grid_resolution=0.001):

        # Calculate the number of grid cells required
        min_x, min_y = np.min(flat_3D_points[:, :2], axis=0)
        max_x, max_y = np.max(flat_3D_points[:, :2], axis=0)
        
        # Determine grid width and height
        grid_width = int(np.ceil((max_x - min_x) / grid_resolution))
        grid_height = int(np.ceil((max_y - min_y) / grid_resolution))
        

        # Create empty occupancy grid
        occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.float32)

        # Map the 3D points to grid cells and update the occupancy grid
        grid_x_indices = ((flat_3D_points[:, 0] - min_x) / grid_resolution).astype(int)
        grid_y_indices = ((flat_3D_points[:, 1] - min_y) / grid_resolution).astype(int)
        # Ensure indices are within bounds
        valid_indices = (grid_x_indices >= 0) & (grid_x_indices < grid_width) & \
                        (grid_y_indices >= 0) & (grid_y_indices < grid_height)

        # Set corresponding grid cells to 1 (occupied)
        occupancy_grid[grid_y_indices[valid_indices], grid_x_indices[valid_indices]] = 1

        return occupancy_grid, min_x, min_y



    def create_suction_kernel(self, triangle_vertices, suction_radius, grid_resolution=0.001, save=False):

        # Convert the suction radius in grid units
        suction_radius_grid = suction_radius / grid_resolution
        
        # Calculate the bounding box of the triangle vertices (in metric units)
        min_x, min_y = np.min(triangle_vertices[:, :2], axis=0)
        max_x, max_y = np.max(triangle_vertices[:, :2], axis=0)
        
        # Extend the bounding box to cover the suction radius (in metric units)
        min_x -= suction_radius
        min_y -= suction_radius
        max_x += suction_radius
        max_y += suction_radius
        
        # Convert the bounding box to grid units
        kernel_width = int(np.ceil((max_x - min_x) / grid_resolution))
        kernel_height = int(np.ceil((max_y - min_y) / grid_resolution))

        # Create an empty kernel
        kernel = np.zeros((kernel_height, kernel_width), dtype=np.float32)

        # Convert triangle vertices from metric coordinates to grid indices
        triangle_x_indices = ((triangle_vertices[:, 0] - min_x) / grid_resolution).astype(int)
        triangle_y_indices = ((triangle_vertices[:, 1] - min_y) / grid_resolution).astype(int)
        
        # Mark grid cells within the suction radius of each triangle vertex
        for (x_idx, y_idx) in zip(triangle_x_indices, triangle_y_indices):
            for i in range(kernel_height):
                for j in range(kernel_width):
                    # Calculate the distance from the grid cell to the triangle vertex
                    distance = np.sqrt((i - y_idx) ** 2 + (j - x_idx) ** 2)
                    if distance <= suction_radius_grid:
                        kernel[i, j] = 1

        kernel /= np.sum(kernel)
        # Save kernel to reuse
        if save:
            np.save('kernel.npy', kernel)

        return kernel


    def choose_approach_point(self, occupancy_grid, suction_kernel, grid_min_X, grid_min_Y, grid_resolution, coverage_threshold=0.85):
        
        # Check if the Region of Interest is atleast equal to or bigger than the suciton cup
        if occupancy_grid.shape[0] >= suction_kernel.shape[0] and occupancy_grid.shape[1] >= suction_kernel.shape[1]:

            # Convolve the occupancy grid with the kernel
            suction_affordance = convolve(occupancy_grid, suction_kernel, mode='constant', cval=0)
            
            # Find the point with the highest suction affordance
            max_coverage_index = np.unravel_index(np.argmax(suction_affordance), suction_affordance.shape)
            self.plot_affordance(suction_affordance, max_point=max_coverage_index)

            if suction_affordance[max_coverage_index] > coverage_threshold:

                X_real_world, Y_real_world = self.grid_to_world_coordinates(max_coverage_index[1], max_coverage_index[0], grid_min_X, grid_min_Y, grid_resolution)
                print(f'Suction Gripper point {(X_real_world, Y_real_world)} found. Please compute Z-coordiante using the plane model you got from RANSAC.')

                return (X_real_world, Y_real_world)
            else:
                return print('Suction cup can not be used for given object. HERE')
        else:
            return print('Suction cup can not be used for given object.')
        
    def grid_to_world_coordinates(self, grid_x, grid_y, grid_min_X, grid_min_Y, grid_resolution):

        print(grid_x, grid_x * grid_resolution)
        print(grid_y, grid_y * grid_resolution)
        x = grid_min_X + (grid_x * grid_resolution)
        y = grid_min_Y + (grid_y * grid_resolution)

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

        # Plot the occupancy grid
        plt.imshow(grid, cmap='gray', interpolation='nearest')

        # Add a colorbar (optional)
        plt.colorbar(label='Grid')

        # Set axis labels (optional)
        plt.xlabel('X')
        plt.ylabel('Y')

        # Show the plot
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

    def generate_suction(self):
        points = np.load('pcl_object_2.npy')
        points = points.T

        pcd = o3d.geometry.PointCloud()

        pcd.points = o3d.utility.Vector3dVector(points)

        background_plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)

        # inlier_cloud = pcd.select_by_index(inliers)
        flat_area_pcd = pcd.select_by_index(inliers, invert=True)

        flat_area_plane_model, _ = flat_area_pcd.segment_plane(distance_threshold=0.008, ransac_n=3, num_iterations=1000)

        triangle_vertices = np.array([[2.5, 4], [2.5, -4], [-4, 0]])/100 # This is garbage, would be better with actual values in meters of course
        suction_radius = 1.25/100  # in meters
        grid_resolution=0.001   # in meters

        occupancy_grid, min_x, min_y = self.generate_occupancy_grid(np.asarray(flat_area_pcd.points))
        self.plot_grid(occupancy_grid)
        occupancy_grid = cv2.erode(occupancy_grid, np.ones((4, 4), np.uint8), iterations=1)
        occupancy_grid = cv2.dilate(occupancy_grid, np.ones((4, 4), np.uint8), iterations=1)
        kernel = self.create_suction_kernel(triangle_vertices, suction_radius, grid_resolution, save=False)
        self.plot_grid(kernel)


        point_xy = self.choose_approach_point(occupancy_grid, kernel, min_x, min_y, grid_resolution, coverage_threshold=0.85)
        print('\n\n\n', point_xy, '\n\n\n')

        if point_xy:
            z = self.compute_z(point_xy, flat_area_pcd)
            point = np.array([point_xy[0], point_xy[1], z])
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01)
            frame.translate([0, 0, z])


            sphere_radius = 0.005 # meters
            mesh_sphere1 = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
            mesh_sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)


            mesh_sphere1.paint_uniform_color([0.5, 0, 0.5])
            mesh_sphere2.paint_uniform_color([0, 0, 0])

            mesh_sphere1.translate([point[0], point[1], point[2]])
            mesh_sphere2.translate([min_x, min_y, point[2]])

            temp_pcd = o3d.geometry.PointCloud()
            temp_pcd.points = o3d.utility.Vector3dVector([point])

            temp_pcd.colors = o3d.utility.Vector3dVector([[1, 0, 0]])

            o3d.visualization.draw_geometries([flat_area_pcd, temp_pcd, mesh_sphere1, frame])

            return point

        else:
            print('No point found')
            return np.array([0, 0, 0])
        
if __name__ == '__main__':
    SuctionGenerator().generate_suction()