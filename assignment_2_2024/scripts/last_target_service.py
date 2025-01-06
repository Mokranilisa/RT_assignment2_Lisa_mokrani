#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float32MultiArray

# Ok, we need to remember the last target position, so let's create some global vars
last_target_x = None
last_target_y = None

def target_callback(msg):
    global last_target_x, last_target_y
    # Everytime we get a new message with target data, we update the last known target
    last_target_x = msg.data[0]
    last_target_y = msg.data[1]
def handle_last_target_request(req):
    global last_target_x, last_target_y
    # Check if we have any target yet, if not we return an error message
    if last_target_x is None or last_target_y is None:
        return TriggerResponse(
            message="No target has been set yet!"  # Yikes! No target set!
        )
    # If there is a target, we return the last target coords
    return TriggerResponse(
        message=f"Last target coordinates: x={last_target_x}, y={last_target_y}"  # Show the coords
    )
def main():
    global last_target_x, last_target_y

    # Let's initialize the ROS node, that's like the "main function" for ROS nodes
    rospy.init_node('last_target_service')

    # We subscribe to the /last_target topic to get the latest target coordinates
    rospy.Subscriber("/last_target", Float32MultiArray, target_callback)

    # Now we create a service to handle requests for the last target. It's like a question and answer session.
    rospy.Service('/last_target', Trigger, handle_last_target_request)

    # Ok, now we just keep it alive with rospy.spin(). Otherwise, it would shut down instantly
    rospy.spin()
if __name__ == "__main__":
    main()

