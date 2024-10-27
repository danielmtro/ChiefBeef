# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.22)

# Define the project name and version
project(gui VERSION 1.0)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "visualization_msgs"
  "sensor_msgs"
  "tf2"
)

set(source_
    "src/Map.cpp"
    "src/GameMap.cpp"
    "src/Credits.cpp"
    "src/MainMenu.cpp"
    "src/Window.cpp"
    "src/Animation.cpp"
    "src/Button.cpp"
    "src/ItemLogger.cpp"
    )

# Include directories
include_directories(include
                    ${OpenCV_INCLUDE_DIRS})

add_executable(Map
               src/Window.cpp
               src/Map.cpp 
               src/GameMap.cpp
               src/Credits.cpp 
               src/MainMenu.cpp
               src/Animation.cpp
               src/Button.cpp
               src/ItemLogger.cpp)

target_compile_definitions(Map PRIVATE MAP_MAIN)

ament_target_dependencies(Map ${dependencies})

# link the sfml library stuff
target_link_libraries(Map sfml-graphics sfml-window sfml-audio sfml-system ${OpenCV_LIBS})

install(TARGETS
  Map
  DESTINATION lib/${PROJECT_NAME})

# Install Fonts directeriory
install(DIRECTORY Fonts/
  DESTINATION share/${PROJECT_NAME}/Fonts
)

# Install Textures directory
install(DIRECTORY Textures/
  DESTINATION share/${PROJECT_NAME}/Textures
)

# Install Music directory
install(DIRECTORY Music/
  DESTINATION share/${PROJECT_NAME}/Music
)

# Handle testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()