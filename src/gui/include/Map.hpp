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

#include <SFML/Graphics.hpp>

class Map : public rclcpp::Node{
    public:

        Map();
        ~Map();

        std::vector<int8_t> map_data_;
        uint32_t width_;   
        uint32_t height_;
        float resolution_;

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

        // map data information that will be received
        
        std::vector<std::vector<int8_t>> map_;

        /**
        * @brief Helper function that resizes a 2D standard vector of ints
        *
        * @param vec is a 2D array of integers that correspond to the map
        * @param width the number of columns in the array
        * @param height the number of rows int he array
        */
        void resize_2D_vector(std::vector<std::vector<int8_t>>& vec, uint32_t width, uint32_t height);
};

#endif