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


//--CLidar Interface-----------------------------------------------------------
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


        // publisher callback function
        void update_scan_data();

        //lidar subscriber
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
        
        // scan_data_publisher
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr scan_data_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stocktake_pub_; 
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Time prev_stocktake_time_;

        //store the current and previous LiDar scan information
        std::vector<double> scan_data_;
        std::vector<double> prev_scan_data_;
        bool is_intense;

        const float DEG2RAD = 3.14159265359/180.0;

        const float stocktake_frequency = 15.0;

        float scan_left[2] = {88*DEG2RAD, 92*DEG2RAD};
        float scan_right[2] = {268*DEG2RAD, 272*DEG2RAD};

        float scan_angle[2] = {90*DEG2RAD, 270*DEG2RAD};


};
#endif