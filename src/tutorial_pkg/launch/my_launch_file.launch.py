from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    my_node = Node(
        package="tutorial_pkg",
        executable="my_first_node",
        name="my_node",
        remappings=[("/image", "/camera/color/image_raw")],
        parameters=[{"timer_period_s": 2}]
    )

    image_saver = Node(
        package="image_view",
        executable="image_saver",
        name="image_saver",
        remappings=[("/image", "/camera/color/image_raw"), ("/camera_info", "/camera/color/camera_info")],
        parameters=[{"save_all_image": False}]
    )

    return LaunchDescription([my_node, image_saver])
