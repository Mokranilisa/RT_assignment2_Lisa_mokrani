README for RT Assignment 2
=============================================

**Name:** Mokrani Lisa  

**ID:** ........

Project Overview
-----------------
This project demonstrates robotic control and state monitoring using **ROS 1**. It includes the following functionalities:

- Sending target positions to the robot using `control.py`.

- Monitoring the last received target through the `last_target_service.py` service.

The system operates in a simulation environment with custom ROS nodes and services.

.. image:: result.gif
   :alt: Demo of the system
   :width: 800


Installation
------------

### Prerequisites

- ROS Noetic on Ubuntu 20.04.
- A configured ROS workspace (`catkin_ws`).
- Python scripts placed in the `scripts/` folder of `assignment_2_2024`.

### Setup Steps

1. **Clone the Repository:**
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Mokranilisa/RT_assignment2_Lisa_mokrani/tree/main
   
   
2. **Make Python Scripts Executable**:
   ```bash
   chmod +x ~/catkin_ws/src/assignment_2_2024/scripts/control.py
   chmod +x ~/catkin_ws/src/assignment_2_2024/scripts/last_target_service.py
   ```

3. **Build the Workspace**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Source the Workspace**:
   ```bash
   source devel/setup.bash
   ```

Running the Simulation
----------------------

### Step 1: Launch the Simulation Environment

1. Start the system with the following command:
   ```bash
   roslaunch assignment_2_2024 assignment1.launch
   ```

2. This will:
   - Launch the simulation from `sim_w1.launch`.
   - Start the following nodes:
     - **`control.py`**: Sends target positions to the robot.
     - **`last_target_service.py`**: Monitors and provides the last received target coordinates.
     - Other existing nodes for wall following, point-to-point movement, and action services.
   
3. Use:

   - To call the node `/last_target`
   ```bash
   rosservice call /last_target
   ```

   - To reveal the x,y position & velocity of the robot, use:
   ```bash
   rostopic list echo /vel_pos
   ```

Features
--------

1. **Control Node (`control.py`)**:

   - Sends user-specified target coordinates to the robot using the `/move_base_simple/goal` topic.
   - Interacts with the simulation to guide the robot to a specified position.

2. **Last Target Service (`last_target_service.py`)**:

   - Subscribes to the `/last_target` topic to record the latest target coordinates sent to the robot.
   - Provides these coordinates via a Trigger service (`/last_target`).
   - Returns a message indicating the last target or a warning if no target has been set.

3. **Simulation Environment**:

   - Robot starts at an initial position (`x=0.0`, `y=1.0`).
   - Coordinates are set as ROS parameters and can be dynamically modified.



