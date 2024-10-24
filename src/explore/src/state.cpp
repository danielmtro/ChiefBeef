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

    // subscribe to the topic
    rotate_request_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/spin_now",
        rclcpp::SensorDataQoS(), 
        std::bind(&State::spin_subscriber, this, std::placeholders::_1)
    );

    // Create Quality of Service for publisher 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Create the publisher for the density of green in the image
    explore_resume_pub_ = this->create_publisher<std_msgs::msg::Bool>("/explore/resume", qos);

    // Create publisher to cmd_vel for 360 degree rotation
    rotate_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);

    // Create timer
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&State::rotate_robot, this));

    // crea

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been initialised.");
}

// deconstructor for the state class
State::~State(){
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 State node has been deleted.");
}

// function that is the callback for beginning the exploration
void State::explore_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Oogway is exploring...");
    const auto &data = msg->data;
    if (data && !has_run_){
        has_run_ = true;
        state_changer("START_EXPLORE");
    }
}

// function that is the callback for beginning the exploration
void State::spin_subscriber(const std_msgs::msg::Bool::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Spinning...");
    const auto &data = msg->data;
    if (data){
        state_changer("START_SCAN");
    }
}

void State::state_changer(std::string state){
    if (state == "START_EXPLORE"){
        RCLCPP_INFO(this->get_logger(), "Oogway is exploring...");
        begin_explore();
    } else if (state == "START_SCAN"){
        RCLCPP_INFO(this->get_logger(), "Oogway is scanning for items...");
        change_explore("PAUSE");
        rotate_robot();
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

    RCLCPP_INFO(this->get_logger(), "Start Spin...");

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

            // set anular velocity to zero - stops moving
            odom_data_.angular.z = 0.0;
            
            rotate_vel_pub_->publish(odom_data_);
        }
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