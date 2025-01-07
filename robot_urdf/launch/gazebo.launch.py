"""
Spawn Robot Description and Launch Boundary Avoidance Node
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='robot_urdf').find('robot_urdf')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot4.xacro')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    # Gazebo Spawn Entity Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description', '-x', '2.0', '-y', '2.0'],
        output='screen'
    )

    # Boundary Avoidance Node
    boundary_avoidance_node = Node(
        package='robot_urdf',  # Replace with your package name
        executable='boundary_avoidance.py',  # Ensure the script is installed as an executable
        name='boundary_avoidance',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        boundary_avoidance_node,  # Add the Boundary Avoidance Node
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'worlds/empty.world', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),
    ])

