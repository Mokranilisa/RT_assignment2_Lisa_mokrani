# RT Assignment 2°: 

Name: Mokrani Lisa.

---

## 📁 Structure

- `assignment_2.ipynb`: Final Jupyter Notebook (located in the `jupyter_notebook` branch)
- `assignment_2_2024/`: ROS package with action server and supporting scripts( description of the package in the main branch)

---

## 🧠 Assignment Overview

This project integrates **ROS** with **Jupyter Notebook** to control a simulated robot and visualize data.

### Part 1: Action Client Interface via Jupyter Notebook

- Set a goal `(x, y)` for the robot using widgets
- Cancel the current goal
- Display robot's:
  - Real-time position and velocity (from `/vel_pos`)
  - Closest obstacle distance (from `/scan`)

### Part 2: Visualization Enhancements

- 📈 **Live trajectory animation** using `FuncAnimation`
- 📊 **Bar chart** showing:
  - Number of successfully reached goals
  - Number of not-reached goals (canceled or failed)

---

## 🚀 How to Run

1. Launch ROS and the simulation:
    ```bash
    roslaunch assignment_2_2024 assignment1.launch
    rosrun assignment_2_2024 bug_as.py
    ```
     This will:
    - Launch the simulation from `sim_w1.launch`.
      
2. Start the Jupyter Notebook:
   ```bash
    jupyter notebook --allow-root
    ```
## ✅ Requirements :

-ROS Noetic
-Gazebo
-Jupyter Notebook
-Python packages: rospy, matplotlib, ipywidgets, numpy
