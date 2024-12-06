# Franka Panda Control

This package demonstrates the control of real Franka Panda Arm.


## Installation

Follow these steps to set up the package:

1. **Download the package**  
   - Download the `franka_panda_hw.zip` file.
   - Extract it to your ROS 1 Noetic workspace's `src` folder.

2. **Build the package**  
   - Open a terminal and navigate to your workspace directory:
     ```bash
     catkin_make
     ```

3. **Source the workspace**  
   - After building, source the setup script:
     ```bash
     source devel/setup.bash
     ```

4. **Prepare the Franka Panda Arm**  
   - Turn On the Franka Panda Arm using the controller PSU (big black box) and wait until the robot's base light turns: YELLOW
   - Open browser and go to address: https://172.16.0.2/desk/  (Note: You may see a warning about the connection being unsafe. Use the "Advanced" option to proceed.)
   - Unlock the robot's joints. The robot's base light will turn WHITE.
   - Click the menu icon (two horizontal lines) in the top-right corner. Select Activate FCI mode.
   - Continuously press the External Enabling device button. The robot's base light will turn BLUE. (You must keep pressing this button to enable robot movement via commands.)

5. **Launch the Franka ROS1**  
   - To start the Franka Panda and ROS1 connection, run:
     ```bash
     roslaunch panda_hw start_hw_robot.launch
     ```
    - This should load the robot in RViz without any environmental objects.

6. **Run the panda control file**  
   - To move the robot to a desired pose:
    - Open new terminal. Source it.
        ```bash
        source devel/setup.bash
        ```
    - Run the control script: 
        ```bash
        rosrun panda_hw panda_control.py
        ```
   - You can make changes in [panda_control.py](/src/panda_hw/src/panda_control.py) at line 192 to move the robot to different location.
   - Note: The planning scene doesn't account for the end effector connection, so adjust coordinates accordingly.
   - Workspace dimensions (in meters):
     - Length: [-0.313, 0.313]
     - Width: [-0.235, 0.235]
     - Maximum Distance from Camera to Table: 0.6
     - The camera origin is considered at its center.
   - In case of Emergency, release the External Enabling device button that will stop the Robot immediately.

7. **Shutdown Procedure**  
   - Stop all the launch file.
   - Release the External Enabling device button.
   - Deactivate FCI mode.
   - Lock the joints using the Lock button in the "Joints" section.(Wait until all joints are locked and clicking sounds stop.)
   - Shut down the robot via the Shutdown button in the top-right menu.
   - Once the fans are off (no sound), turn off the controller PSU (big black box).
