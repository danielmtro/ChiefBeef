/*
teleop.hpp

This file holds the definition of the Teleop class.
This is a class that allows the manual driving of the turtlebot

Written by Tumali Embuldeniya, 2024
*/

#ifndef TELEOP_H
#define TELEOP_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>


class Teleop : public rclcpp::Node {
    public:
        Teleop();

    private:
        void teleop_run();
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};


#endif