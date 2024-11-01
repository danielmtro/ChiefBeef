"""

"""

#include "explore/dummy.hpp"
#include "explore/explore.h"
#include <thread>
#include <chrono>
#include <stdexcept>

Delay::Delay() : Node("delaying") {
    // subscribe to the topic "/slam_request" -> published by the GUI
    slam_request_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/slam_request",
        rclcpp::SensorDataQoS(), 
        std::bind(&Delay::explore_subscriber_callback, this, std::placeholders::_1)
    );
}

// function that begins the explore node.
void Delay::explore_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg){
    try {
        // Launch ROS 2 state node in a separate thread
        std::thread ros_launch_thread([]() {
            std::string command = "ros2 launch explore_lite explore.launch.py use_sim_time:=false";
            
            int result = std::system(command.c_str());
            if (result != 0) {
                std::cerr << "Failed to launch the ROS 2 launch file. Exit code: " << result << std::endl;
                throw std::runtime_error("Launch file execution failed.");
            } else {
                std::cout << "Launch file called successfully!" << std::endl;
            }
        });

        // Ensure the thread joins properly
        ros_launch_thread.join();
    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught in run_nav: " << e.what() << std::endl;
        std::terminate();
    }
}

#ifdef DELAY_MAIN

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        std::thread ros_launch_thread([]() {
          std::string command = "ros2 launch explore_lite state.launch.py use_sim_time:=false";
            
          std::cout << "Attempting to execute command: " << command << std::endl; // Debug message
          int result = std::system(command.c_str());

          if (result != 0) {
              std::cerr << "Failed to launch the state ROS 2 launch file. Exit code: " << result << std::endl;
              throw std::runtime_error("State launch file execution failed.");
          } else {
              std::cout << "State launch file called successfully!" << std::endl;
          }
        });
      // Spin the ROS node while waiting for operations
      rclcpp::spin(std::make_shared<Delay>());

      // Ensure the thread joins properly
      ros_launch_thread.join();
      
    } catch (const std::exception& e) {
        std::cerr << "Exception caught in main: " << e.what() << std::endl;
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return 0;
}

#endif