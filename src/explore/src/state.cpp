/*
state.cpp

This file contains the implementation of the  state class

Written by James Hocking, 2024
*/

#include "explore/state.hpp"
#include "explore/explore.h"
#include <thread>


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

    RCLCPP_INFO(this->get_logger(), "RECEIVED DATA");

    if (data && !has_run_){
        has_run_ = true;
        
        // start the explore node
        auto node = std::make_shared<explore::Explore>();

        // Launch a separate thread for ROS2
        std::thread ros_thread([&]() {
            rclcpp::spin(node);
        });

    }
}

// function to run the nodes for the slam
void State::launch_nodes(){
    RCLCPP_INFO(this->get_logger(), "Beginning Slam");
}