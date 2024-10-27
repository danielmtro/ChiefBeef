/*
map.hpp INTERFACE

This is the interface for the GUI that will be used to visualise the 
map and see the robot.

Subscribes to the /map topic that is provided by SLAM

Written: Daniel Monteiro
Date: 12/10/2024
*/

#ifndef _MAP_HPP
#define _MAP_HPP

#include <vector>
#include <memory>
#include <cmath>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <SFML/Graphics.hpp>
#include "ItemLogger.hpp"
#include "constants.hpp"

class Map : public rclcpp::Node{
    public:

        //  Struct to hold info about the robots
        struct Pose {
            double x, y, z;           // Position
            double roll, pitch, yaw;   // Orientation (RPY angles)
        };

        // struct to hold informatino about the map data
        struct MapMetaData
        {
            float resolution, width, height;
            float o_x, o_y; // origin
        };

        Map();
        ~Map();

        // Sets new_map_available_ to be false
        void read_map_data();

        // getters
        uint32_t get_width() const;
        uint32_t get_height() const;
        float get_resolution() const;
        std::vector<int8_t> get_map() const;
        bool get_map_available() const; 

        /**
        * @brief this function is called every time a message is received from
        * /map topic and it stores the map information
        */
        void publish_slam_request();

        // item logger should be accessible to everything
        std::shared_ptr<ItemLogger>  get_item_logger();

        // current pose retrieval
        Pose get_current_pose() const;
        MapMetaData get_map_meta_data() const;
        Pose get_map_pose() const;
        float get_battery_percentage() const;

        /**
        * @brief transforms the input map data to a new array in the 
        * original space based on the quaternion that the map is oriented at
        */
        void transform_map_orientation();

    private:

        MapMetaData map_meta_data_;
    
        std::shared_ptr<ItemLogger> item_logger_;
        
        /**
        * @brief this function is called every time a message is received from
        * /map topic and it stores the map information
        *
        * @param msg is an 32 bit integer which represents the current state 
        */
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        /**
        * @brief this function is called every time a message is received from
        * /map topic and it stores the map information
        *
        * @param msg is a list of integer messages that corresponds to april tags in the current frame
        */
        void item_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

        /**
        * @brief function takes in the robots odometry and stores the pose
        *
        * @param msg a nav_msgs::msg::Odometry::SharedPtr that corresponds to a quaternion (4 dimensional)
        */        
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
        * @brief function takes in the robots battery and stores the percentage
        * @param msg a sensor_msgs::msg::Batterystate::SharedPtr that contains info about the battery
        */        
        void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

        // subscriber for the current map that has been created
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

        // Variables to store inputs received from msg
        std::vector<int8_t> map_data_;
        std::vector<int8_t> transformed_map_;

        uint32_t width_;   
        uint32_t height_;
        float resolution_;

        // boolean to determine if new map data is ready for reading
        bool new_map_available_;

        /**
        * @brief Helper function that resizes a 2D standard vector of ints
        *
        * @param vec is a 2D array of integers that correspond to the map
        * @param width the number of columns in the array
        * @param height the number of rows int he array
        */
        void resize_2D_vector(std::vector<std::vector<int8_t>>& vec, uint32_t width, uint32_t height);

        // slam publisher
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slam_publisher_;

        // items subscriber
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr item_subscriber_;

        // odometry subscriber
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // battery state subscriber
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
        
        // tracks the current pose of the robot
        Pose current_pose_;
        Pose current_map_pose_;

        // tracks the codes that have already been observed
        std::unordered_set<int> codes_seen_;

        // battery
        float battery_percentage_;
};

#endif