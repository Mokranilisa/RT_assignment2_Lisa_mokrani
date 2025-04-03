control Module
==============

This module controls the robot's movement and goal setting. It listens to the `/odom` topic to get the robot's current position and velocity, and it sends goals to the `/reaching_goal` action server. Additionally, it publishes the robot's position and velocity to the `/vel_pos` topic.

## Overview

- **Subscribes to:** `/odom` (Odometry messages with position and velocity).

- **Publishes to:** `/vel_pos` (Custom message with position & velocity), `/last_target` (Last goal position).

- **Uses an Action Client** to send movement goals to `/reaching_goal`.

---

## ?? Functions

### **1?? publisher(msg)**

- **Purpose:** Processes Odometry messages and publishes position & velocity.

- **Input:** `msg (Odometry)`: Contains robot's position and velocity.  
- **Publishes to `/vel_pos`:**  

  - `cor_x (float)`: X coordinate of the robot's position. 
   
  - `cor_y (float)`: Y coordinate of the robot's position.
    
  - `vel_x (float)`: Velocity in the X direction.  
  
  - `vel_y (float)`: Velocity in the Y direction.  

.. code-block:: python

   def publisher(msg):
       """
       Callback function for Odometry messages.

       Args:
           msg (Odometry): The Odometry message with position and velocity data.
       """
       position = msg.pose.pose.position
       velocity = msg.twist.twist.linear

       vel_pos_msg = VelPos()
       vel_pos_msg.cor_x = position.x
       vel_pos_msg.cor_y = position.y
       vel_pos_msg.vel_x = velocity.x
       vel_pos_msg.vel_y = velocity.y

       pub.publish(vel_pos_msg)

---

### **2?? action_client()**

- **Purpose:** Sends movement goals to the `/reaching_goal` action server. 
 
- **How it works:**  

  1. The user enters target coordinates (`X, Y`).  
  2. The function sends these as a goal to the **ROS action server**.  
  3. The robot moves towards the target.  
  4. If the user types `'c'`, the goal is canceled.  
  
- **Publishes to `/last_target`:** Stores the last target coordinates.

.. code-block:: python

   def action_client():
       """
       Action client to send goals to the /reaching_goal action server.

       Allows the user to input target coordinates and sends them as goals.
       """
       client = actionlib.SimpleActionClient('reaching_goal', GoalType)
       client.wait_for_server()

       while not rospy.is_shutdown():
           x = input("Enter target x coordinate: ")
           y = input("Enter target y coordinate: ")

           goal = GoalType()
           goal.target_x = float(x)
           goal.target_y = float(y)

           client.send_goal(goal)
           rospy.loginfo(f"Goal sent: ({x}, {y})")

           # Publish last target position
           last_target_msg = Float32MultiArray()
           last_target_msg.data = [float(x), float(y)]
           last_target_pub.publish(last_target_msg)

           if input("Cancel goal? (y/n): ") == 'y':
               client.cancel_goal()
               rospy.loginfo("Goal canceled.")

---

### **3?? main()**

- **Purpose:** Initializes the ROS node and connects all components.  

- **Sets up:**  

  - **Subscribers**: Listens to `/odom` for position updates. 
   
  - **Publishers**: Sends velocity & last target position.  
  
  - **Action Client**: Sends movement goals.  
  
- **Flow:**  

  1. The script starts the ROS node.  
  2. It listens for position updates from `/odom`.  
  3. It calculates and publishes movement data to `/vel_pos`.  
  4. It waits for user input to send new goals via `/reaching_goal`.  

.. code-block:: python

   def main():
       """
       Initializes the ROS node, sets up subscribers, and action clients.
       """

       rospy.init_node('control_node')

       # Publisher for /vel_pos
       vel_pos_pub = rospy.Publisher('/vel_pos', VelPos, queue_size=10)
       # Publisher for /last_target
       last_target_pub = rospy.Publisher('/last_target', Float32MultiArray, queue_size=10)

       # Subscriber for /odom
       rospy.Subscriber('/odom', Odometry, publisher)

       # Action client for goal sending
       action_client()

       rospy.spin()

---

## ?? Example Usage
To use the control module, run:

```bash
rosrun my_robot_package control.py



