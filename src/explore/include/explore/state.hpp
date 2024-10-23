/*
state.hpp

This file holds the definition of the State class.
This is a high level class that waits for the GUI to 
begin the SLAM - initialising the process.

Written by James Hocking, 2024
*/

#ifndef STATE_H
#define STATE_H


#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include <iostream>

/*
State

This class waits for the GUI to initialise 
the SLAM - and launches all of the required nodes
for the slam to occur.
*/
class State : public rclcpp::Node {
    public:
        // constructor for the State class 
        State();

        // deconstructor for the State class
        ~State();
    private:
        // subscriber to the slam_request
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slam_request_sub_;

        // publisher to the "explore/resume" topic
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_resume_pub_;

        /**
         * @brief This function is the callback for the subscription. It
         * checkes whether the nodes are already running and if not launches them.
         */
        void explore_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg);

        /**
         * @brief This function changes the current state of the robot, whether 
         * it is navigating, scanning, or being idle.
         */
        void state_changer(std::string new_state);

        /**
         * 
         * @brief This function is either pause or unpause the exploring. We will
         * use this to spin at random points to view everything.
         */
        void change_explore(std::string to_change);

        /**
         * @brief This function begins a node that starts the exploration.
         */
        void begin_explore();

        /**
         * @brief Function to rotate the robot 360 degrees.
         */
        void rotate_robot();

        // boolean that keeps track if already ran
        bool has_run_;

        // current pose of the robot
        float pose_;
};

#endif