# Slam Code
The SLAM code will hold three main components:
- `SLAM`. This will create a map of the local environment 
- `NAV2` - This creates the paths and moves the bot based on a goal.
- `EXPLORE_LITE` - This finds frontiers to explore and sends it back to nav2.

## Running explore_lite 
1) Build all the packages (not just turt3_gazebo)
   ```bash
   colcon build --symlink-install && source install/setup.bash && . /usr/share/gazebo/setup.sh && ros2 launch turt3_gazebo launch.launch.py
   ```
2) Open RVIZ, nav and slam_toolbox - all in different terminals
   ```bash
   ros2 launch turtlebot3_bringup rviz2.launch.py 
   ```
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
   ```
   ```bash
   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
   ```
3) Run the explore.cpp file to move the turtlebot around
   ```bash
   ros2 launch explore_lite explore.launch.py
   ```
4) Open RVIZ and go to:  Add > By Topic > Map > /map
     This opens the mapping function in RVIZ so that you can see it being built                 

## Compile the package
```bash
colcon build --packages-select tutorial_pkg
```
