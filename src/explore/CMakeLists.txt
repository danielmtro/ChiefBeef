cmake_minimum_required(VERSION 3.5)
project(explore_lite)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set flag depending on distro
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "eloquent")
  message(STATUS "Build for ROS2 eloquent")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DELOQUENT")
elseif("$ENV{ROS_DISTRO}" STREQUAL "dashing")
  message(STATUS "Build for ROS2 dashing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDASHING")
else()
  message(STATUS "BuilD for ROS2: " "$ENV{ROS_DISTRO}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(apriltag_ros REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  map_msgs
  nav2_costmap_2d
  visualization_msgs
  apriltag_ros
)

include_directories(
    include
)

install(
  DIRECTORY include/explore/
  DESTINATION include/explore/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


add_executable(explore
  src/delay.cpp
  src/explore.cpp
  src/costmap_client.cpp
  src/state.cpp
  src/frontier_search.cpp
  src/lidar.cpp
  src/teleop.cpp
)

add_executable(blank
  src/explore.cpp
  src/costmap_client.cpp
  src/state.cpp
  src/frontier_search.cpp
  src/delay.cpp
  src/teleop.cpp
)

add_executable(state
  src/state.cpp
)

add_executable(lidar
  src/lidar.cpp
)

add_executable(teleop
  src/teleop.cpp
)

target_include_directories(lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(lidar PRIVATE LIDAR_MAIN)
target_link_libraries(lidar ${rclcpp_LIBRARIES})
ament_target_dependencies(lidar ${DEPENDENCIES})

install(TARGETS
  lidar
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(blank PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Define STATE_MAIN for the blank target
target_compile_definitions(blank PRIVATE EXPLORE_MAIN)

target_link_libraries(blank ${rclcpp_LIBRARIES})
ament_target_dependencies(blank ${DEPENDENCIES})

install(TARGETS blank
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(explore PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(explore PRIVATE MAIN_MAIN)


target_link_libraries(explore ${rclcpp_LIBRARIES})

ament_target_dependencies(explore ${DEPENDENCIES})

install(TARGETS explore
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Define STATE_MAIN for the blank target
target_compile_definitions(state PRIVATE STATE_MAIN)

target_link_libraries(state ${rclcpp_LIBRARIES})
ament_target_dependencies(state ${DEPENDENCIES})

install(TARGETS state
  DESTINATION lib/${PROJECT_NAME})


target_include_directories(teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(teleop PRIVATE TELEOP_MAIN)

target_link_libraries(teleop ${rclcpp_LIBRARIES})
ament_target_dependencies(teleop ${DEPENDENCIES})

install(TARGETS teleop
  DESTINATION lib/${PROJECT_NAME}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


ament_export_include_directories(include)
ament_package()