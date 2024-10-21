/*
state.cpp

This file contains the implementation of the  state class

Written by James Hocking, 2024
*/

#include "explore/state.hpp"
#include "explore/explore.h"
#include <thread>
#include <stdexcept>

std::atomic<bool> explore_running(false); // Flag to control the thread
std::atomic<bool> cam_running(false); // Flag to control the thread

void run_nav() {
    try {
        // Launch ROS 2 state node in a separate thread
        std::thread ros_launch_thread([]() {
            std::string command = "ros2 launch explore_lite explore.launch.py use_sim_time:=true";
            
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


// constructor for the state class
State::State() : Node("state"), has_run_(false) {
    std::cout << "State Node has been created." << std::endl;

    // subscribe to the topic
    slam_request_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/slam_request",
        rclcpp::SensorDataQoS(), 
        std::bind(&State::subscriber_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been initialised.");
}

// deconstructor for the state class
State::~State(){
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been deleted.");
}


void State::subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg){
    const auto &data = msg->data;
    if (data && !has_run_){
        has_run_ = true;
        state_changer("START_EXPLORE");
    }
}

// TODO: make a subscriber to the camera class to start the state machine with START_SCAN

void State::state_changer(std::string state){
    if (state == "START_EXPLORE"){
        RCLCPP_INFO(this->get_logger(), "Oogway is exploring...");
        explore_running = true;
        cam_running = true;
        run_nav();

        // start the camera node to check when to jump into scan


    } else if (state == "START_SCAN"){
        RCLCPP_INFO(this->get_logger(), "Oogway is scanning for items...");
        explore_running = false;

        // logic for spin

        // after complete, recall function with exploration
        this->state_changer("START_EXPLORE");


    } else if (state == "FINISH"){

        RCLCPP_INFO(this->get_logger(), "Oogway has finished scanning. Have a nice day!");
        
        // end the explore node, end the camera node
        explore_running = false;
        cam_running = false;

    } else {
        RCLCPP_INFO(this->get_logger(), "INVALID STATE. Check your code.");
    }
}

# ifdef STATE_MAIN
# define STATE_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
#endif