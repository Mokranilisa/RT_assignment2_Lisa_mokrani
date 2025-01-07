# RT Assignment 2°: 

Name: Mokrani Lisa.

ID: ............

- This branch `main` represents the part 1 of the second RT assignement.
- Switch branch to `ROS2` to access to part 2 of the assignement 

The **part 1** demonstrates showcasing robotic control and state monitoring using custom scripts in **ROS 1**. The project includes the following functionalities:

1. **Sending target positions to the robot using `control.py` also publishing costum message (cor_x, cor_y, vel_x, vel_y) as position & velocity.**
2. **Monitoring the last target received through the `last_target_service.py` service.**

The system is implemented to interact seamlessly in a simulation environment, using custom ROS nodes and services as shown below:
<p align="center">
<img src="result.gif" alt="Demo of the system" width="800">
</p>

---

## Installation

Follow these steps to install and set up the project:

### Prerequisites
- ROS Noetic installed on Ubuntu 20.04.
- A properly configured ROS workspace (`catkin_ws`).
- Python scripts placed in the `scripts/` folder of the package `assignment_2_2024`.

---

### Steps
1. **Clone the Repository**:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/Mokranilisa/RT_assignment2_Lisa_mokrani/tree/main
    ```

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

---

## Running the Simulation

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
   
    - To reveal the x,y position & velocity of the robot use:

    ```bash

    rostopic list echo /vel_pos

    ```
---

## Features

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

---

## Node Descriptions

### 1. **Control Node (`control.py`)**
- **Input**:
    - Target positions specified by the user (e.g., `x=3.0`, `y=5.0`).
- **Output**:
    - Publishes the goal to `/reaching_goal/goal`.
    - Confirms goal sent.

---

### 2. **Last Target Monitoring Service (`last_target_service.py`)**
- **Input**:
    - Subscribes to `/last_target` for the most recent coordinates.
- **Output**:
    - Returns the last target coordinates when requested through the `/last_target` service.
