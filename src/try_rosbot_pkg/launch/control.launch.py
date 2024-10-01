import os

from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument
#from launch.conditions import IfCondition
#from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lidar_control = Node(
        package='try_rosbot_pkg',
        executable='lidar_control_node',
        name='lidar_control'
    )

    range_control = Node(
        package='try_rosbot_pkg',
        executable='range_control_node',
        name='range_control'
    )

    camera_control = Node(
        package='try_rosbot_pkg',
        executable='camera_control_node',
        name='camera_control'
    )
    
    tracker = Node(
        package='try_rosbot_pkg',
        executable='tracker_node',
        name='tracker'
    )

    movement_control = Node(
        package='try_rosbot_pkg',
        executable='movement_control_node',
        name='movement_control'
    )

    ld = LaunchDescription()

    ld.add_action(lidar_control)
    ld.add_action(range_control)
    ld.add_action(camera_control)
    ld.add_action(tracker)
    ld.add_action(movement_control)

    return ld
