from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_node = Node(
        package="tutorial_pkg",
        executable="lidar_control_node",
        name="lidar_node",
        parameters=[{"timer_period_s": 2}]
    )

    return LaunchDescription([lidar_node])
