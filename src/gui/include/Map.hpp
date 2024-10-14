/*
map.hpp INTERFACE

This is the interface for the GUI that will be used to visualise the 
map and see the robot.

Subscribes to the /map topic that is provided by SLAM

Written: Daniel Monteiro
*/

#ifndef _MAP_HPP
#define _MAP_HPP

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <SFML/Graphics.hpp>
#include "constants.hpp"

class Map : public rclcpp::Node{
    public:

        Map();
        ~Map();

        // Sets new_map_available_ to be false
        void read_map_data();
        bool get_map_available() const; 

        // getters
        uint32_t get_width() const;
        uint32_t get_height() const;
        float get_resolution() const;
        std::vector<int8_t> get_map() const;

        /**
        * @brief this function is called every time a message is received from
        * /map topic and it stores the map information
        */
        void publish_slam_request();

    private:
        
        /**
        * @brief this function is called every time a message is received from
        * /map topic and it stores the map information
        *
        * @param msg is an 32 bit integer which represents the current state 
        */
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        // subscriber for the current map that has been created
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

        // Variables to store inputs received from msg
        std::vector<int8_t> map_data_;
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
};

#endif