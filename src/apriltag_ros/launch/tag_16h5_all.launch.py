import launch
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        # Get the package share directory for apriltag_ros
    apriltag_share_dir = get_package_share_directory('apriltag_ros')

    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/image_rect", "/camera/image_raw"), ("/camera_info", "/camera/camera_info")],
        parameters=[os.path.join(apriltag_share_dir, 'cfg', 'tags_36h11.yaml')])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
