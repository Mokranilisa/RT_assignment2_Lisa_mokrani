#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class BoundaryAvoidance(Node):
    def __init__(self):
        super().__init__('boundary_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Define boundaries
        self.threshold = 8.0
        self.get_logger().info("Boundary avoidance node started.")

    def odom_callback(self, msg):
        # Get the robot's current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Create a Twist message to control the robot
        twist = Twist()

        # Check if the robot is within bounds
        if abs(x) <= self.threshold and abs(y) <= self.threshold:
            # Robot is within bounds, move straight
            twist.linear.x = 2.0  # Move forward
            twist.angular.z = 0.0  
            print(f"Position: x={x:.2f}, y={y:.2f} ")
            print("Moveing Forwards")
        else:
            # Robot is out of bounds, turn angularly
            twist.linear.x = 1.7  
            twist.angular.z = 2.0  # Rotate in place
            print(f"Position: x={x:.2f}, y={y:.2f} ")
            print("Rotating")

        # Publish the velocity command
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BoundaryAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

