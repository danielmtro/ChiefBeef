
#ifndef WAITFORBUTTON_H
#define WAITFORBUTTON_H

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>


class WaitForButton : public rclcpp::Node {
    public:
        WaitForButton();
        ~WaitForButton();

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