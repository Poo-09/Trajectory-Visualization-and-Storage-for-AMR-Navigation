# Trajectory-Visualization-and-Storage-for-AMR-Navigation

## Overview
This ROS2 package is designed to automate trajectory visualization and storage for Autonomous Mobile Robots (AMR) using the ROS2 framework. It enables users to visualize the trajectory of a robot in RViz and store it in a file for future analysis.

### Features:
- **Trajectory Publisher and Saver Node**:
  - Collects trajectory data while the robot moves.
  - Publishes trajectory as a MarkerArray for visualization in RViz.
  - Provides a ROS service to save trajectory data in JSON, CSV, or YAML format.
  - Allows specifying a time duration to save only recent trajectory data.

- **Trajectory Reader and Publisher Node**:
  - Reads the saved trajectory file.
  - Transforms trajectory data to the `odom` frame.
  - Publishes the transformed trajectory for visualization.

## 1. Package Creation
Run the following command to create the ROS2 package:
```bash
cd ~/ros2_ws/src
ros2 pkg create trajectory_manager --build-type ament_cmake --dependencies rclcpp nav_msgs visualization_msgs geometry_msgs
```
Then, navigate to the package:
```bash
cd ~/ros2_ws/src/trajectory_manager
```

## 2. Custom Service Definition
Define a custom service `SaveTrajectory.srv` inside `srv/` directory:
```bash
mkdir srv
nano srv/SaveTrajectory.srv
```
Add the following content:
```plaintext
string filename
float64 duration
---
bool success
```
Update `CMakeLists.txt` and `package.xml` to include service dependencies.

## 3. Trajectory Publisher and Saver Node
This node collects trajectory data, publishes it as a MarkerArray, and provides a ROS service for saving trajectory data.
Run the following command to launch this node:
```bash
ros2 run trajectory_manager trajectory_publisher_saver
```

## 4. Trajectory Reader and Publisher Node
This node reads a saved trajectory file and republishes it for visualization.
Run it using:
```bash
ros2 run trajectory_manager trajectory_reader_publisher
```

## 5. Configuration
Modify `CMakeLists.txt` and `package.xml` to include required dependencies.

### Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_manager
source install/setup.bash
```

## 6. Testing and Validation
- Launch TurtleBot3 simulation:
  ```bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
- Run the trajectory publisher:
  ```bash
  ros2 run trajectory_manager trajectory_publisher_saver
  ```
- Move the robot using teleop:
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard
  ```
- Visualize trajectory in RViz:
  ```bash
  rviz2
  ```
  - Set **Fixed Frame** to `odom`
  - Add **MarkerArray** and subscribe to `/trajectory_markers`
- Save trajectory data:
  ```bash
  ros2 service call /save_trajectory trajectory_manager/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 10.0}"
  ```
- Read and publish the saved trajectory:
  ```bash
  ros2 run trajectory_manager trajectory_reader_publisher
  ```

## Expected Output
- A complete ROS2 package with:
  - **Trajectory Publisher and Saver Node**
  - **Trajectory Reader and Publisher Node**
  - **Custom ROS Service**
  - **Launch files** for easy execution
- RViz visualization of the robot's trajectory
- Ability to save and reload trajectory data

## Pseudocode for Key Functionalities

### Trajectory Collection and Publishing
```cpp
Subscribe to /odom topic
Initialize empty trajectory list
For each incoming odometry message:
    Extract robot position
    Append position to trajectory list
    Publish trajectory as MarkerArray to RViz
```

### Saving Trajectory Data
```cpp
Receive filename and duration from service request
Extract trajectory data within specified duration
Save data in requested format (CSV, JSON, YAML)
Return success status to the client
```

### Reading and Publishing Saved Trajectory
```cpp
Open trajectory file
Read and parse trajectory data
Transform trajectory points to 'odom' frame
Publish transformed trajectory as MarkerArray
```


