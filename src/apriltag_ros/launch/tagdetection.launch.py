"""
This is the launch file for Project 1 for the Master Oogway team. This 
file starts the world, creates the maze, positions the turtlebot, etc.


"""

# all of the imports 
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_april_tags = get_package_share_directory('apriltag_ros')

    tag_detector_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_april_tags, 'launch', 'tag_16h5_all.launch.py')
        )
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(tag_detector_cmd)
    return ld
