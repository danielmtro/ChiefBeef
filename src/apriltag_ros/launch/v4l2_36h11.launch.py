import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument 'device'
    device_arg = DeclareLaunchArgument(
        name='device',
        default_value='0',
        description='Video device (e.g., 0 for /dev/video0)'
    )

    # Get the package share directory for apriltag_ros
    apriltag_share_dir = get_package_share_directory('apriltag_ros')

    # Define the composable nodes to be launched in the container
    composable_nodes = [
        # AprilTag detection node (apriltag_ros)
        ComposableNode(
            package='apriltag_ros',
            plugin='AprilTagNode',
            name='apriltag',
            namespace='apriltag',
            remappings=[
                ('/apriltag/image_rect', '/camera/image_raw'),
                ('/apriltag/camera_info', '/camera/camera_info')
            ],
            parameters=[os.path.join(apriltag_share_dir, 'cfg', 'tags_36h11.yaml')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    ]

    # Define the node container to run the composable nodes
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        device_arg,
        container
    ])
