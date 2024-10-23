# ChiefBeef
Major Project implementation for a turtlebot within a grocery store. This 
bot will be able to traverse the store using SLAM to localise itself and 
use it's camera to keep a record of stock. A GUI will be used to see the 
current inventory and control the robot.

# How to setup the Physical Turtlebot
1) Turn the Turtlebot on and power up
2) Wait for it to make the sound and for the green light on the second top level to turn on (this is WIFI)
3) Connect to the wifi `eb542147pi`
4) `ssh ubuntu@10.42.0.1` in the terminal to take over the terminal of the bot
5) run `selfTest` to check the turtlebot functionality

For all of the above, any passwords asked is `turtlebot`

# Discord 
https://discord.gg/9uQkYcss


# Running Gazebo Simulations
```bash
colcon build --packages-select turt3_gazebo --symlink-install && source install/setup.bash && . /usr/share/gazebo/setup.sh && ros2 launch turt3_gazebo launch.launch.py


# Running RVIZ
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

# Creating the SLAM from packages
```bash
// In one terminal
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

// In the other terminal
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

# Creating the Navigation from one launch file !!!!
This is what we are trying to get running by tomorrow. The 
command to run this is 
```bash

# Work for Will & James 23/10
1) Merge each of our branches into main
2) Integrate april tag detection into build
3) Check launch of SLAM & camera simultaneously
4) Check SLAM process while running camera
5) Check camera FOV and april tag on angle detection
5) Write stock take process
6) Write lidar scanner for when to stock take