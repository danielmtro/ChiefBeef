/*
map.hpp IMPLEMETATION

This is the interface for the GUI that will be used to visualise the 
map and see the robot.

Subscribes to the /map topic that is provided by SLAM

Written: Daniel Monteiro
Date: 12/10/2024
*/


#include "Map.hpp"
#include "MainMenu.hpp"
#include "Credits.hpp"
#include "GameMap.hpp"


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

    // create a logger to store recorded items
    item_logger_ = std::make_shared<ItemLogger>();

    item_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "items",
        qos_settings,
        std::bind(                  
        &Map::item_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1));

    // create the slam publisher
    slam_publisher_ = this->create_publisher<std_msgs::msg::Bool>("slam_request", 10);

}

Map::~Map()
{
    RCLCPP_INFO(this->get_logger(), "Map Node has been terminated");
} 

std::shared_ptr<ItemLogger> Map::get_item_logger()
{
    return item_logger_;
}

void Map::item_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // add the current item to the item logger
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    item_logger_->add_item(msg->data.c_str());    
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

void Map::publish_slam_request()
{
    std_msgs::msg::Bool message;
    message.data = true;
    slam_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(),"Published SLAM request");

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
    int selection = 0;

    while(selection != EXIT)
    {
        mainMenu.RunMenu();
        selection = mainMenu.get_menu_selection();
        std::cout << "Selection was " << selection << std::endl;

        if(selection == SHOPPING_TIME) {
            // create a gamemap window based on the ROS2 node    
            GameMap gmap(GmapWindow::MAP_NAME, GmapWindow::MAP_WIDTH, GmapWindow::MAP_HEIGHT, node);
            gmap.RunMap();

        }
        else if(selection == MEET_THE_TEAM)
        {
            Credits credits(CreditsWindow::MAP_NAME, CreditsWindow::MAP_WIDTH, CreditsWindow::MAP_HEIGHT);
            credits.RunCredits();
        }
    }

    // Clean up ROS2 when SFML window is closed
    rclcpp::shutdown();
    ros_thread.join();

    return 0;
}
