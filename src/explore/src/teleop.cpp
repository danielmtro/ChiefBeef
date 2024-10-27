/*
teleop.cpp

This file contains the implementation of the  teleop class

Written by Tumali Embuldeniya, 2024
*/

#include "teleop.hpp"


Teleop::Teleop() : Node("teleop") 
{
    teleop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Teleop Node Started. Use WASD to control the robot.");

    teleop_run();
}


void Teleop::teleop_run()
{
    char user_input_;
    geometry_msgs::msg::Twist vel;

    std::cout << "Enter input (WASD - to drive, X - to quit): ";
    std::cin >> user_input_;

    while (user_input_ != "x")
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;

        switch(user_input_)
        {
            case 'w':
                vel.linear.x = 0.5;
                break;
            case 's';
                vel.linear.x = -0.5;
                break;
            case 'a';
                vel.angular.z = 0.5;
                break;
            case 'd';
                vel.angular.z = -0.5;
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command! (Use 'WASD' to drive, 'X' to quit).");
                continue;
        }

        teleop_pub_->publish(vel);

        std::cout << "Enter input (WASD - to drive, X - to quit): ";
        std::cin >> user_input_;
    }
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

