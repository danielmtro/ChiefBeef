/*
teleop.cpp

This file contains the implementation of the  teleop class

Written by Tumali Embuldeniya, 2024
*/

#include "explore/teleop.hpp"


Teleop::Teleop() : Node("teleop") 
{
    teleop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Teleop Node Started. Use WASD to control the robot.");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                     std::bind(&Teleop::teleop_run, this));
}


void Teleop::teleop_run()
{
    std::string user_input_;
    geometry_msgs::msg::Twist vel;

    std::cout << "Enter input (WASD - to drive, X - to quit): ";
    std::cin >> user_input_;

    if (user_input_ == "x") {
        rclcpp::shutdown(); // Exit if "x" is entered
        return;
    }

    if (user_input_.compare("w") == 0)
    {
        vel.linear.x = 0.2;
    }
    else if(user_input_.compare("s") == 0)
    {
        vel.linear.x = -0.2;
    }
    else if(user_input_.compare("a") == 0)
    {
        vel.angular.z = 0.2;
    }
    else if(user_input_.compare("d") == 0)
    {
        vel.angular.z = -0.2;
    }
    else if(user_input_.compare("q") == 0)
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }
    else
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        RCLCPP_WARN(this->get_logger(), "Unknown command! (Use 'WASD' to drive, 'X' to quit).");
    }

    teleop_pub_->publish(vel);
}

#ifdef TELEOP_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<Teleop>()); 
  rclcpp::shutdown();
  return 0;
}
#endif

