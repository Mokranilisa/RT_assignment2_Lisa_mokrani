last_target_service Module
==========================

This module implements a ROS service that provides the last target position. It listens to the `/last_target` topic to update the last known target coordinates, and it exposes a service `/last_target` that clients can use to retrieve the last target coordinates.

## Overview

- **Subscribes to:** `/last_target` (Float32MultiArray messages with target coordinates).

- **Exposes Service:** `/last_target` (Service that returns the last known target coordinates).

- **Publishes to:** None (does not publish any data).

- **Uses ROS Service** to provide the last target position to clients.

---

## ?? Functions

### **1?? target_callback(msg)**

- **Purpose:** Processes incoming messages on the `/last_target` topic to update the robot's last target position.

- **Description:**  
  Every time a new message containing the target position (X, Y coordinates) is received, this callback function updates the global `last_target_x` and `last_target_y` variables.
  
- **Parameters:**
  - `msg (Float32MultiArray)`: Contains the X and Y coordinates of the target.

.. code-block:: python

   def target_callback(msg):
       """
       Callback function for processing target coordinates from the /last_target topic.

       Args:
           msg (Float32MultiArray): The message containing target X, Y coordinates.
       """
       global last_target_x, last_target_y

       last_target_x = msg.data[0]  # X coordinate
       last_target_y = msg.data[1]  # Y coordinate

---

### **2?? handle_last_target_request(req)**

- **Purpose:** Handles service requests to return the last known target position.

- **Description:**  
  When a service request is made, this function checks if a target has been set. If a target has been set, it returns the last target position (X, Y) as a string. If no target has been set, it returns an error message.
  
- **Parameters:**
  - `req (Trigger)`: The request message (not used in this implementation).
  
- **Returns:**
  - A `TriggerResponse` with the last known target coordinates or an error message if no target has been set.

.. code-block:: python

   def handle_last_target_request(req):
       """
       Handles the service request to return the last known target coordinates.

       Args:
           req (Trigger): The service request (not used).

       Returns:
           TriggerResponse: The last target coordinates or an error message if no target has been set.
       """
       if last_target_x is not None and last_target_y is not None:
           response = TriggerResponse()
           response.success = True
           response.message = f"Last target coordinates: ({last_target_x}, {last_target_y})"
           return response
       else:
           response = TriggerResponse()
           response.success = False
           response.message = "No target set yet."
           return response

---

### **3?? main()**

- **Purpose:** Initializes the ROS service node and subscribes to the `/last_target` topic to update the target coordinates. It also exposes a service `/last_target` to retrieve the last target.

- **Description:**  
  This function starts the ROS service, subscribes to the `/last_target` topic to listen for incoming target coordinates, and exposes the `/last_target` service for clients to call and retrieve the last target coordinates.
  
- **Subscribes to:**
  - `/last_target` topic to receive updated target coordinates.
  
- **Exposes:**
  - `/last_target` service to retrieve the last target position.

.. code-block:: python

   def main():
       """
       Initializes the ROS service node, subscribes to /last_target, and provides a service to retrieve the last target position.
       """
       rospy.init_node('last_target_service_node')

       # Subscribe to the /last_target topic to get target coordinates
       rospy.Subscriber('/last_target', Float32MultiArray, target_callback)

       # Provide the service for clients to get the last target position
       last_target_service = rospy.Service('/last_target', Trigger, handle_last_target_request)

       rospy.spin()

---

## ?? Example Usage

To use the `last_target_service` module, run the `last_target_service.py` script:

```bash
rosrun my_robot_package last_target_service.py



