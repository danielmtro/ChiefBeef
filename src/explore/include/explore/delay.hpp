"""

"""

#ifndef DELAY_H
#define DELAY_H

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>


class Delay : public rclcpp::Node {
    public:
        // constructor for the Delay class
        Delay();

        // deconstructor for the Delay class
        ~Delay();

    private:
        // subscriber to the slam_request
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slam_request_sub_;

        /**
         * @brief This function is the callback for the subscription. It
         * checkes whether the nodes are already running and if not launches them.
         */
        void explore_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

#endif