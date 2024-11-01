"""
Probs not for final submission
"""


import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the current package
    slam_toolbox_dir = FindPackageShare('slam_toolbox')

    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "slam_params.yaml"
    )

    # Declare whether to use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    # Include the SLAM Toolbox launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_toolbox_dir, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time, "slam_params_file":config}.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            slam_launch,
        ]
    )
