/*
state.cpp

This file contains the implementation of the  state class

Written by James Hocking, 2024
*/

#include "explore/state.hpp"
#include "explore/explore.h"
#include <thread>
#include <chrono>
#include <stdexcept>

// constructor for the state class
State::State() : Node("state"), has_run_(false) {
    // subscribe to the topic /spin_now -> published by the Lidar Node
    rotate_request_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/spin_now",
        rclcpp::SensorDataQoS(), 
        std::bind(&State::spin_subscriber_callback, this, std::placeholders::_1)
    );

    // Create Quality of Service for publisher 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Create the publisher for the density of green in the image
    explore_resume_pub_ = this->create_publisher<std_msgs::msg::Bool>("/explore/resume", qos);

    // Create publisher to cmd_vel for 360 degree rotation
    rotate_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been initialised.");
}

// deconstructor for the state class
State::~State(){
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been deleted.");
}

// function that is a callback for beginning a spin
void State::spin_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg){
    const auto &data = msg->data;
    if (data){
        state_changer("START_SCAN");
    }
}

// function to change the state of the state machine
void State::state_changer(std::string state){
    if (state == "START_SCAN"){
        RCLCPP_INFO(this->get_logger(), "Oogway is scanning for items...");
        change_explore("PAUSE");
        rotate_robot();
        RCLCPP_INFO(this->get_logger(), "Oogway is continuing to explore...");
        change_explore("RESUME");
    } else if (state == "FINISH"){
        RCLCPP_INFO(this->get_logger(), "Oogway has finished scanning. Have a nice day!");
        change_explore("PAUSE");
    } else {
        RCLCPP_INFO(this->get_logger(), "INVALID STATE. Check your code.");
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

    // find the total time to turn 360 degrees with set ang_vel_
    turn_time_ = 2 *  M_PI / ang_vel_;

    // get the start time of rotation
    start_time_ = this->get_clock()->now();

    // get odom data of current orientation
    auto odom_data_ = geometry_msgs::msg::Twist();


    while(true){

        // get current time
        auto current_time_ = this->get_clock()->now();

        // calculate the elapsed time
        elapsed_time_ = (current_time_ - start_time_).seconds();

        // check if the turn is complete
        if (elapsed_time_ < turn_time_){

            //set angular velocity to the desired value - keeps it turning
            odom_data_.angular.z = ang_vel_;

            // publish odom_data_ to "/cmd_vel"
            rotate_vel_pub_->publish(odom_data_);
        } else{
            break;
        }
    }
}

#ifdef STATE_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<State>()); 
  rclcpp::shutdown();
  return 0;
}
#endif

