/*
map.hpp IMPLEMETATION

This is the interface for the GUI that will be used to visualise the 
map and see the robot.

Subscribes to the /map topic that is provided by SLAM

Written: Daniel Monteiro
*/


#include "Map.hpp"


Map::Map() : Node("Map_Node")
{
    // Create QoS settings for suscriber - only retain the latest map update
    rclcpp::QoS qos_settings(rclcpp::KeepLast(1));
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // initialise the data vector
    map_ = std::vector<std::vector<int8_t>>();
    
    // create the map subscriber function
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        qos_settings,
        std::bind(                  
        &Map::map_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

}

Map::~Map()
{
    RCLCPP_INFO(this->get_logger(), "Map Node has been terminated");
} 

/*
Map callback receives a map message, stores the relative information 
and continues onwards
*/
void Map::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // extract relevant data information
    resolution_ = msg->info.resolution;
    width_ = msg->info.width;
    height_ = msg->info.height;
    map_data_ = msg->data;
    
    // resize the current map size
    resize_2D_vector(map_, width_, height_);
    
    RCLCPP_INFO(this->get_logger(), "Received occupancy grid with width: %d, height: %d", width_, height_);

}

/*
Function resizes a 2D array to be a given size
*/
void Map::resize_2D_vector(std::vector<std::vector<int8_t>>& vec, uint32_t width, uint32_t height)
{
    // Resize the outer vector to the desired number of rows
    vec.resize(height);

    // Resize each inner vector to the desired number of columns
    for (size_t i = 0; i < height; ++i) {
        vec[i].resize(width);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Map>());
    rclcpp::shutdown();
    return 0;
}