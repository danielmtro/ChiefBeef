/*
state.cpp

This file contains the implementation of the  state class

Written by James Hocking, 2024
*/

#include "explore/state.hpp"
#include "explore/explore.h"
#include <thread>
#include <stdexcept>

// constructor for the state class
State::State() : Node("state"), has_run_(false) {
    std::cout << "State Node has been created." << std::endl;

    // subscribe to the topic
    slam_request_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/slam_request",
        rclcpp::SensorDataQoS(), 
        std::bind(&State::explore_subscriber_callback, this, std::placeholders::_1)
    );

    // Create Quality of Service for publisher 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Create the publisher for the density of green in the image
    explore_resume_pub_ = this->create_publisher<std_msgs::msg::Bool>("/explore/resume", qos);

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been initialised.");
}

// deconstructor for the state class
State::~State(){
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been deleted.");
}

// function that is the callback for beginning the exploration
void State::explore_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg){
    const auto &data = msg->data;
    if (data && !has_run_){
        has_run_ = true;
        state_changer("START_EXPLORE");
    }
}

void State::state_changer(std::string state){
    if (state == "START_EXPLORE"){
        RCLCPP_INFO(this->get_logger(), "Oogway is exploring...");
        begin_explore();
    } else if (state == "START_SCAN"){
        RCLCPP_INFO(this->get_logger(), "Oogway is scanning for items...");
        change_explore("PAUSE");
        // start a function call here to spin 360 deg.
        change_explore("RESUME");
    } else if (state == "FINISH"){
        RCLCPP_INFO(this->get_logger(), "Oogway has finished scanning. Have a nice day!");
        change_explore("PAUSE");
    } else {
        RCLCPP_INFO(this->get_logger(), "INVALID STATE. Check your code.");
    }
}

// function that begins the explore node.
void State::begin_explore() {
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

// function for publishing to pause explore
void State::change_explore(std::string to_change) {
    std_msgs::msg::Bool explore_msg;
    if (to_change == "PAUSE"){
        explore_msg.data = false;
    } else if (to_change == "RESUME"){
        explore_msg.data = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error trying to change explore to a state that doesn't exist.");
        return;
    }
    explore_resume_pub_->publish(explore_msg);
} 

// function to rotate the turtlebot
void State::rotate_robot(){
    float initial_pose = pose_;

    float added_angle = 0;

    while (added_angle < 360) {
        
    }
}

# ifdef STATE_MAIN
# define STATE_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<explore::Explore>()); 
  rclcpp::shutdown();
  return 0;
}
#endif