# Robot-diff-ros2

This project involves the simulation and visualization of a custom robot using ROS 2, RViz, and Gazebo. The project is divided into two main packages: `my_robot_description` for the robot's URDF model and RViz visualization, and `my_robot_bringup` for launching the robot in Gazebo.

## Packages

### 1. `my_robot_description`

This package contains the URDF description of the robot and launch files to visualize it in RViz.

#### Key Files:
- **URDF Files**: Define the robot's structure.
- **Launch Files**: Includes `display.launch.xml` to visualize the robot in RViz.

#### Launching RViz:

To visualize the robot's URDF model in RViz, use the following command:

```
ros2 launch my_robot_description display.launch.xml
```

### 2. `my_robot_bringup`

This package is responsible for bringing up the robot in Gazebo and includes configurations for launching RViz alongside Gazebo.

#### Key Files:
- **Gazebo World: Contains a custom world for simulating the robot.
- **Launch Files: Includes `my_robot_gazebo.launch.xml` to launch the robot in Gazebo with RViz.

#### Launching Gazebo

To launch the robot in Gazebo and RViz, use the following command:
```
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

## Installation and setup

1. Build the packages:
   ```
   colcon build --packages-select my_robot_description my_robot_bringup --symlink-install
   ```

2. Source the environment:
   ```
   source install/setup.bash
   ```

## Running the simulations

### 1. Visualize in RViz
To view the robot's URDF model in RViz, use:

```
ros2 launch my_robot_description display.launch.xml
```

### 2. Simulate in Gazebo
To launch the robot in Gazebo along with RViz, use:

```
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

## Controlling the robot

### Control the robot arm
To add parameters to control the robot arm, publish a joint trajectory using the following command:
```
ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {frame_id: "basefoot_link"}, joint_names: ["arm_base_forearm_joint", "forearm_hand_joint"], points: [ {positions: {0.0, 0.3}} ]}'
```
P.S: Feel free to change the positions as per your experiment

### Control the arm
To control the mobile robot's movement, use the following command to publish velocity commands:

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```
P.S: Feel free to change the linear as well as angular, as per your experiment


## Troubleshooting
- **Build Errors: Ensure all dependencies are correctly installed and sourced by running `source /install/setup.bash`.
- **RViz Issues: Verify that the RViz configuration file paths are correct in the launch files.
- **Gazebo Launch Failures: Check for missing dependencies or incorrect file paths in the launch file.

## License
License
This project is licensed under the [License](LICENSE). See the LICENSE file for more details.
