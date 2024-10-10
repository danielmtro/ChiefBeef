/*
map.hpp IMPLEMETATION

This is the interface for the GUI that will be used to visualise the 
map and see the robot.

Subscribes to the /map topic that is provided by SLAM

Written: Daniel Monteiro
*/


#include "Map.hpp"
#include "GameMap.hpp"
#include "MainMenu.hpp"


Map::Map() : Node("Map_Node")
{
    // Create QoS settings for suscriber - only retain the latest map update
    rclcpp::QoS qos_settings(rclcpp::KeepLast(1));
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
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

    // process that we've read the current map
    new_map_available_ = true;
    
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

void Map::read_map_data()
{
    new_map_available_ = false;
}

/*

Getters available below

*/

bool Map::get_map_available() const
{
    return new_map_available_;
}


uint32_t Map::get_width() const
{
    return width_;
}

uint32_t Map::get_height() const
{
    return height_;
}

float Map::get_resolution() const
{
    return resolution_;
}

std::vector<int8_t> Map::get_map() const
{
    return map_data_;
}

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create ROS2 node
    auto node = std::make_shared<Map>();

    // Launch a separate thread for ROS2
    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    // create a menu window
    MainMenu mainMenu(MenuWindow::MENU_WINDOW_NAME, MenuWindow::MENU_WIDTH, MenuWindow::MENU_HEIGHT);

    // stay in the main menu until a selecion is ready
    mainMenu.RunMenu();
    int selection = mainMenu.get_menu_selection();
    std::cout << "Selection was " << selection << std::endl;

    if(selection == 0) {
        // create a gamemap window based on the ROS2 node    
        GameMap gmap(GmapWindow::MAP_NAME, GmapWindow::MAP_WIDTH, GmapWindow::MAP_HEIGHT, node);
        gmap.RunMap();
    }

    // Clean up ROS2 when SFML window is closed
    rclcpp::shutdown();
    ros_thread.join();

    return 0;
}
