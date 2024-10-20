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

        /**
         * @brief This function launches all the nodes neccessary for 
         * the SLAM to run.
         */
        void launch_nodes();

        /**
         * @brief This function is the callback for the subscription. It
         * checkes whether the nodes are already running and if not launches them.
         */
        void subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg);

        // boolean that keeps track if already ran
        bool has_run_;
};

#endif