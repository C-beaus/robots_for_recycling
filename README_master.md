# WPI RBE 595 Autonomous Recycling

This repository contains an implementation of autonomous robotic recyling using a conveyor belt system and two robotic arms. Various grippers can be attached to both arms for better grasping of certain waste. The project contains a classification model to identify and sort waste into five classes, including **cardboard**, **glass**, **metal**, **paper**, and **plastic**. More information about the classification implementation can be found in the *README_classification.md* file in this repository.

Once waste items are classified, we identify grasps on each waste object and attempt to execute each grasp with a *PANDA* robotic arm. The robotic arm will then attempt to properly sort each waste item by its class (identified with the classification model). Any waste item the arm can not pick up will be filtered after attempted execution. Once the arm has attempted to grasp all waste items in the camera frame, the remaining waste items will be processed by the cartesian robot. Any waste that the cartesian robot can not grasp will be deemed unsortable by the current system and moved into a **miscellaneous** group by running off the end of the conveyor belt.

## Usage

To run the code, first clone this repository into the src folder of your catkin workspace (ensure you are using ROS1). Next, create a virtual environment in the workspace running **python 3.8.10**. Activate the virtual environment in the command line and run the following command to install the required packages: 
```
"pip install -r requirements.txt"
```

To run the code, open a new terminal and navigate to the catkin workspace where the robots_for_recycling package is located. Catkin make and source this workspace by running the following: 
```
"catkin_make"
``` 
```
"source devel/setup.bash"
```
Next, run the following command to launch all necessary nodes in the project: 
```
roslaunch robots_for_recycling start.launch"
```

To start the PANDA nodes, open a new terminal and navigate to the "panda_recycling" workspace. Catkin make and source this workspace, then run the following in the command line to launch all necessary nodes in the project: 
```
"roslaunch panda_hw start.launch"
```

The project will run continuously until the user performs a manual kill command by typing control C (^C) in the terminal where these launch files were executed.

