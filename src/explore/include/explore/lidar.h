/*
lidar.h

This file holds the definition of the Lidar class.
This class uses the lidar readings to detect spikes in
intensity, which then triggers a stock take to occur.

Written by William Ridley-Smith, 2024
*/
#ifndef _LIDAR_H_
#define _LIDAR_H_


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


/*
Lidar

This class gets the scan data from the lidar, and publishes the intensity
as well as an instruction whether or not a stock take should take place.
*/
class Lidar : public rclcpp::Node{
    public:
        Lidar();
        ~Lidar();

        // getter for the scan data
        std::vector<double> get_scan_data();

    private:
        /**
         * @brief this function is the callback for when @lidar_sub_ receives
         * a message from a topic. This function stores the relavent scan angle
         * data into scan_data_ as well as moving the old scan_data into the 
         * prev_scan_data_ member variable
         * 
         * @param msg is the message received via the topic which lidar_sub
         * is subscribed to
         */
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        /**
         * @brief This is the publisher callback function
         */
        void update_scan_data();

        // Lidar subscriber
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
        
        // Publisher for the scan data, for debugging
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr scan_data_pub_;

        // Stock take publisher to indicate when to scan for tags
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stocktake_pub_; 

        // Timer for getting the lidar data, as well as checking when a 
        // stocktake can occur
        rclcpp::TimerBase::SharedPtr update_timer_;

        // The previous time at which a stock take was triggered
        rclcpp::Time prev_stocktake_time_;

        // Vectors to store the current and previous lidar scan information
        std::vector<double> scan_data_;
        std::vector<double> prev_scan_data_;

        // Boolean for whether or not the reading from the lidar indicates 
        // a spike in intensity readings
        bool is_intense_;

        // Conversion for degrees to radians
        const float DEG2RAD = 3.14159265359/180.0;

        // This is the minimum amount of time between stocktakes in seconds
        const float stocktake_frequency_ = 100.0;

        // To trigger a stocktake, the intensity value must be greater than this
        const float intensity_threshold_ = 6500;
        // Intensity must be this number times greater to trigger a stock take
        const float change_threshold_ = 1.50;

        // Range of angles for scanning the left and right side of the robot
        const float scan_left_[2] = {88*DEG2RAD, 92*DEG2RAD};
        const float scan_right_[2] = {268*DEG2RAD, 272*DEG2RAD};

        // The exact left and right side of the robot
        const float scan_angle_[2] = {90*DEG2RAD, 270*DEG2RAD};
};
#endif