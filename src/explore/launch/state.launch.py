"""
state.launch.py

This file brings up slam_toolbox and the nav stack.
The delay in between the launch ensures that the previous node is 
running before it begins.

James Hocking, 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    explore_dir = FindPackageShare('explore_lite')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([explore_dir, 'config', 'params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    # Adding a delay before launching the SLAM node
    slam_delay = TimerAction(
        period=2.0,  # Delay in seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([explore_dir, 'launch', 'slam.launch.py'])
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
        ]
    )

    # Adding a delay before launching the navigation nodes
    nav2_bringup_delay = TimerAction(
        period=2.0,  # Delay in seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            )
        ]
    )

    return LaunchDescription(
        [
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            slam_delay,
            nav2_bringup_delay,
        ]
    )
