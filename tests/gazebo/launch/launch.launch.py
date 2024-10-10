"""

Launches the world file...
James Hocking, 2024

"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turt3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

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

    world = WorldFile()

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
            'Y': world.robot_yaw,
        }.items()
    )

    # get the walls
    maze_model_file = os.path.join(
        get_package_share_directory('turt3_gazebo'),
        'worlds', world.maze_name, 'model.sdf'
    )

    spawn_walls_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_walls',
        arguments=[
            '-entity', 'walls',
            '-file', maze_model_file,
            '-x', world.walls_pos[0], '-y', world.walls_pos[1], '-z', world.walls_pos[2]
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
    return ld

""" 
EDIT THE FOLLOWING TO CHANGE THE POSITION OF THE MAZE WITHIN THE WORLD.
PLEASE DONT PUSH THESE CHANGES UNLESS THEY ARE GOOD. 
(Eg if you are testing at different spots feel free to change but 
dont push your changes.)
"""
world_data = {
    "Double-Box": {
        "walls_pos":["-1", "-1", "0"],
        "robot_yaw": "-3.141"
    },
    "Simple-Box" : {
        "walls_pos":["2", "-1", "0"],
        "robot_yaw": "0"
    }, 
    "Two-Shapes-of-Box": {
        "walls_pos":["-1", "2", "0"],
        "robot_yaw": "0"
    }
}


class WorldFile:
    def __init__(self):
        # create the input string
        input_string = "What Maze do you want? Input a number:\n\n"
        for i, option in enumerate(world_data.keys()):
            input_string += f"{i+1}. : {option}\n"
        input_string += "\n"

        self.maze_id = int(input(input_string)) - 1

        # keep asking till one that is allowed is given
        while self.maze_id < 0 or self.maze_id > len(world_data.keys()):
            print("Bad id, try again:\n")
            self.maze_id = int(input()) - 1

        # get the rest of the worlds attributes
        self.maze_name = list(world_data.keys())[self.maze_id]
        data_dict = world_data[self.maze_name]

        self.walls_pos = data_dict["walls_pos"]
        self.robot_yaw = data_dict["robot_yaw"]


