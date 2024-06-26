import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_params_file = LaunchConfiguration('urdf_params_file')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("try_rosbot_pkg"), 'config', 'slam.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )
    
    """
    urdf = DeclareLaunchArgument(
        'urdf_params_file',
        default_value=os.path.join(
            get_package_share_directory("try_rosbot_pkg"), 'urdf', 'rosbot_macro.urdf.xacro'
        ),
        description='Full path to the ROS2 parameters file to use for rosbot urdf',
    )
    
    """
    urdf_path = os.path.join(get_package_share_directory("try_rosbot_pkg"), 'urdf', 'rosbot.urdf.xacro')
    #urdf_path = "/home/husarion/ros2_ws/src/try_rosbot_pkg/urdf/rosbot_macro.urdf.xacro"
    #urdf = open(urdf_path).read()
    urdf = xacro.process_file(urdf_path).toxml()
    

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    urdf_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}]
        #parameters=[{
        #	'robot_description': ParameterValue(
        #		Command(['xacro ', str(urdf_path)]), value_type=str
        #	)
        #}],
    )
    
    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)

    ld.add_action(slam_node)
    ld.add_action(urdf_node)

    return ld
