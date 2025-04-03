.. assignment_2_2024 documentation master file, created by
   sphinx-quickstart on Tue Mar 25 13:42:09 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.
   
Welcome to assignment_2_2024's documentation!
=============================================

Real-Time Robotics Control System
---------------------------------

This project is a real-time robotic control system that manages movement and tracks the last known target position.  
It is designed to work with **ROS (Robot Operating System)** and consists of two main components:

- **`control.py`**: Computes and publishes velocity commands for the robot.
- **`last_target_service.py`**: Stores and provides the last known target position.

.. note::

   This documentation explains the implementation details and how to use the system.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   control
   last_target_service

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Modules Documentation
=====================

Control Module
--------------

This module handles the robot control and goal setting.

.. automodule:: control
   :members:
   :undoc-members:
   :show-inheritance:
   
Last Target Service Module
--------------------------

This module provides a ROS service to retrieve the last target.

.. automodule:: last_target_service
   :members:
   :undoc-members:
   :show-inheritance:




