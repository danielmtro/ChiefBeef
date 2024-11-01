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

# Running The Turtlebot
There are 3 Steps for this. Make sure to do it in order!
1) Run in the ssh terminal both `bringup` in one terminal and `camera` in another
2) Once they are setup, run in your local terminal after colcon building and sourcing. Wait for a while 
until you see blue text saying "Creating bond timer"
```bash
ros2 run explore_lite explore
```
3) Run the GUI using the below. Press the stocktake button to start.
``` bash
ros2 run gui Map
```
4) Optionally, to visualise the turtlebots state in RVIZ.
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```


# Running Gazebo Simulations
```bash
colcon build --packages-select turt3_gazebo --symlink-install && source install/setup.bash && . /usr/share/gazebo/setup.sh && ros2 launch turt3_gazebo launch.launch.py
```

# Creating the SLAM from packages
In one terminal
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
In the other terminal
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

# Installing April Tag Packages
```bash
sudo apt-install ros-humble-apriltag
sudo apt-install ros-humble-apriltag-msgs
```
