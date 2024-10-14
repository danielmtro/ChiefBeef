cmake_minimum_required(VERSION 3.5)
project(tutorial_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(explore_lite REQUIRED)
find_package(slam_toolbox REQUIRED)

# Install the launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
