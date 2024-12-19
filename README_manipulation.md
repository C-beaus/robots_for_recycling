# WPI RBE 595 Autonomous Recycling (Manipulation)

This file will explain more in depth the functionality of the `antipodal_planner` and `suction_planner` nodes.

## Antipodal Planner

The `antipodal_planner` was adapted from a previous paper, [*Antipodal Robotic Grasping using Generative Residual Convolutional Neural Network*](https://arxiv.org/abs/1909.04810), by Sulabh Kumra, Shirin Joshi, Ferat Sahin. Their code repository can be found [here](https://github.com/skumra/robotic-grasping).

The planner utilizes a pre-trained Generative Residual Convolutional Neural Network (GR-ConvNet) on the Cornell Grasping Dataset to predict suitable antipodal grasps from an image of objects.

First, a request is made from the `task_planner` node to generate grasps. This information comes from the *select_grasps_from_bbs* service as an array of bounding boxes.

The planner also utilizes the *run_grasp_model* service, using the RGBD image taken by the RealSense camera in the *camera_node* in the inference.

For each bounding box, inference is run on the object to determine the highest quality grasp, returning the [$x$, $y$, $z$, $\theta$, width] of each grasp in meters from the camera frame.

The planner finally responds to the *select_grasps_from_bbs* service with the flattened grasp poses, sent and reshaped in the `task_planner` node.

## Suction Planner

The main purpose of the `suction_planner` is to idenitfy flat surfaces of objects that are suitable for a suction cup manipulator. This utilizes [Open3D](https://www.open3d.org/docs/release/index.html) to handle point cloud generation and processing.

First, a request is made from the `task_planner` node to generate suction points. This information comes from the *get_sucked* service as an array of bounding boxes and a depth map in the camera frame.

There are two different methods used to identify flat surfaces, broken down into a "Slow Suction Method" and "Fast Suction Method."

### Slow Suction Method

This method first iterates through each bounding box, generating the most suitable suction point for each object. The planner crops the image to the bounding box of the object, crops the conveyor belt, then creates a pointcloud using the existing camera intrinsics.

The pointcloud is filtered by estimating surface normals for each point where the z-component is greater than a certain threshold. This helps to identify flat regions on the object.

The pointcloud is then transformed into an occupancy grid

### Fast Suction Method



### Conclusion

For each object, the suction methods return the [$x$, $y$, $z$] for the highest affordance point of each location in meters from the camera frame.

The planner finally responds to the *get_sucked* service with the suction poses, sent and reshaped in the `task_planner` node.