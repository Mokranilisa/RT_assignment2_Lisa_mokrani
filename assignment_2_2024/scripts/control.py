#!/usr/bin/env python

"""
Control Module

This module controls the robot's movement and sends goals to the action server.

Author: Lisa Mokrani
Date: March 2025
"""

import rospy
import actionlib
from assignment_2_2024.msg import PlanningGoal, PlanningAction
from assignment_2_2024.msg import vel_pos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from std_msgs.msg import Float32MultiArray

def publisher(msg):
"""
    Publishes the robot's velocity and position from the Odometry message.

    Args:
        msg (Odometry): The Odometry message containing position and velocity data.

    Publishes:
        vel_pos: A custom message containing:
            - cor_x (float): X position.
            - cor_y (float): Y position.
            - vel_x (float): Linear velocity in X direction.
            - vel_y (float): Linear velocity in Y direction.
    """
    global pub
    # Get the position information from the Odometry message
    pos = msg.pose.pose.position
    # Get the velocity information from the Odometry message
    velocity = msg.twist.twist.linear
    # Now we create the custom message (vel_pos)
    velpos = vel_pos()
    # We assign values to the vel_pos message
    velpos.cor_x = pos.x
    velpos.cor_y = pos.y
    velpos.vel_x = velocity.x
    velpos.vel_y = velocity.y
    # Publish the message to /vel_pos so that other nodes cann use it
    pub.publish(velpos)
    
# Action client function to interact with the /reaching_goal action server
def action_client():
"""
    Action client function to send goal positions to the `/reaching_goal` action server.

    Continuously asks for user input to set target positions or cancel goals.

    Publishes:
        Float32MultiArray: The last target position for tracking.
    """
    global target_pub
    # Here we create the action client that will talk to /reaching_goal action server
    action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    # We wait for the server to be ready before sending any goals
    action_client.wait_for_server()
    while not rospy.is_shutdown():
        print("Please enter the target position or type 'c' to cancel:")
        x_input = input("X position of target: ")
        y_input = input("Y position of target: ")
        if x_input == "c" or y_input == "c":
            # If user wants to cancel the goal, we cancel it
            action_client.cancel_goal()
            print("Goal cancelled.")
        else:
            try:
                # Convert the user input to float
                last_target_x = float(x_input)
                last_target_y = float(y_input)
                # Publish the last target coordinates so other nodes can know about them
                target_pub.publish(Float32MultiArray(data=[last_target_x, last_target_y]))
                # Now we create a goal message and assign the target coordinates to it
                goal = PlanningGoal()
                goal.target_pose.pose.position.x = last_target_x
                goal.target_pose.pose.position.y = last_target_y
                # Send the goal to the action server
                action_client.send_goal(goal)
                print(f"Goal sent: x={last_target_x}, y={last_target_y}")
            except ValueError:
                # If the input is not a number, we'll tell the user to enter numeric values
                print("Invalid input. Please enter numeric values.")

# Main function to initialize everything
def main():
"""
    Initializes the ROS node, publishers, and subscribers.

    - Publishes velocity and position to `/vel_pos`
    - Publishes last target position to `/last_target`
    - Subscribes to `/odom` to receive Odometry messages
    - Starts the action client for goal setting
    """
    global pub, target_pub
    # Start the ROS node
    rospy.init_node('control')
    # Publisher to send position and velocity to /vel_pos
    pub = rospy.Publisher("/vel_pos", vel_pos, queue_size=1)
    # Publisher to send the last target to /last_target
    target_pub = rospy.Publisher("/last_target", Float32MultiArray, queue_size=1)
    # Subscriber to listen to the /odom topic and send the data to publisher function
    rospy.Subscriber("/odom", Odometry, publisher)
    # Now we call the action client function to start sending goals
    action_client()

if __name__ == "__main__":
    # This will run the main function to start the whole process
    main()




