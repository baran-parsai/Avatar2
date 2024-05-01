import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    imagery= '/home/jenkin/Documents/avatar/Avatar2/ros_avatar'   # where the imagery lives

    return LaunchDescription([
        Node(
             package='avatar2',
             executable='ros_avatar',
             name='ros_avatar',
             output='screen',
             parameters=[{'imagery': imagery}]),
    ])

