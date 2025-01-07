
# Assignemnet 2 Part 2

Welcome to `ROS2` branch. In case you would like to access **Part1** branch switch to `main`.

This part 2 ROS 2 package implements a simple behavior for a robot to move around in the environment. The node subscribes to odometry data from the `/odom` topic, checks if the robot's position is within predefined boundaries, and publishes velocity commands to the `/cmd_vel` topic to keep the robot within the allowed area.

<p align="center">
<img src="result1.gif" alt="Demo of the system" width="900">
</p>
---

## Features

- **Boundary Detection**: The node monitors the robot's position and ensures it stays within a square boundary defined by `±8.0` units in both the `x` and `y` directions.
- **Proactive Behavior**:
  - If the robot is within bounds, it moves straight forward at a constant linear velocity.
  - If the robot approaches or exceeds the boundaries, it rotates to return to the allowed area.
- **Customizable Threshold**: The boundary threshold is set to `8.0` by default, but it can be modified in the code.

---

## Topics

### Subscribed Topics:
- **`/odom`**: 
  - Message Type: `nav_msgs/Odometry`
  - Used to read the robot's current position in the environment.

### Published Topics:
- **`/cmd_vel`**:
  - Message Type: `geometry_msgs/Twist`
  - Used to control the robot's linear and angular velocities.

---

## How It Works

1. The robot's position is monitored using data from the `/odom` topic.
2. If the robot's position stays within the defined boundary (e.g., `-8.0 <= x, y <= 8.0`), the robot moves forward in a straight line.
3. If the robot's position crosses the boundary, it stops moving forward and rotates in place to return to the allowed area.
4. Velocity commands are continuously published to the `/cmd_vel` topic.

---

## Usage

### Prerequisites:
- ROS 2 installed on your system.
- A robot or simulation environment (e.g., Gazebo) publishing to `/odom` and subscribing to `/cmd_vel`.

### Steps:
1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   ```
2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```
4. Run the node:
   ```bash
   ros2 run <package_name> boundary_avoidance
   ```

---

## Parameters

- **`threshold`**: Defines the size of the square boundary around the origin (`x=0, y=0`) that the robot must stay within. Default is `8.0`.

---
## Author

Mokrani Lisa
ID: 
