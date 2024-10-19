
import os
import time  # Import time for sleep functionality
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turt3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_april_tags = get_package_share_directory('apriltag_ros')

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define the world file
    world = os.path.join(
        get_package_share_directory('turt3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    # Gazebo server command
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client command
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher command
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    # Create an instance of WorldFile
    world_file = WorldFile()
    
    # Launch configuration for robot spawn
    x_pose = LaunchConfiguration('x_pose', default="0.00")
    y_pose = LaunchConfiguration('y_pose', default="0.00")

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'robot_yaw': world_file.robot_yaw,
        }.items()
    )

    # Get the walls
    maze_model_file = os.path.join(
        get_package_share_directory('turt3_gazebo'),
        'worlds', world_file.maze_name, 'model.sdf'
    )

    tag_detector_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_april_tags, 'launch', 'tag_16h5_all.launch.py')
        )
    )

    # Spawn walls command
    spawn_walls_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_walls',
        arguments=[
            '-entity', 'walls',
            '-file', maze_model_file,
            '-x', world_file.walls_pos[0], '-y', world_file.walls_pos[1], '-z', world_file.walls_pos[2]
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_walls_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(tag_detector_cmd)

    # Add a delay to ensure services are available
    time.sleep(2)  # You can adjust this delay if necessary

    return ld

class WorldFile:
    def __init__(self):
        # Create the input string
        input_string = "What Maze do you want? Input a number:\n\n"
        for i, option in enumerate(world_data.keys()):
            input_string += f"{i+1}. : {option}\n"
        input_string += "\n"

        self.maze_id = int(input(input_string)) - 1

        # Keep asking till one that is allowed is given
        while self.maze_id < 0 or self.maze_id >= len(world_data.keys()):  # Changed condition to include last index
            print("Bad id, try again:\n")
            self.maze_id = int(input()) - 1

        # Get the rest of the world's attributes
        self.maze_name = list(world_data.keys())[self.maze_id]
        data_dict = world_data[self.maze_name]

        self.walls_pos = data_dict["walls_pos"]
        self.robot_yaw = data_dict["robot_yaw"]

# Maze data
world_data = {
    "Double-Box": {
        "walls_pos": ["-1", "-1", "0"],
        "robot_yaw": "-3.141"
    },
    "Simple-Box": {
        "walls_pos": ["2", "-1", "0"],
        "robot_yaw": "0"
    },
    "Two-Shapes-of-Box": {
        "walls_pos": ["-1", "2", "0"],
        "robot_yaw": "0"
    },
    "Double-Box-New-Tags": {
        "walls_pos": ["-0.5", "1.5", "0.3"],
        "robot_yaw": "1.5707963268"
    }
}