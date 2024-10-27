/*
teleop.hpp

This file holds the definition of the Teleop class.
This is a class that allows the manual driving of the turtlebot

Written by Tumali Embuldeniya, 2024
*/

#ifndef TELEOP_H
#define TELEOP_H


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

class Teleop : public rclcpp::Node 
{
    public:
        Teleop();

    private:
        void teleop_run();
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
}


#endif