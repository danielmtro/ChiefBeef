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
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create ROS2 node
    auto node = std::make_shared<Map>();

    // Launch a separate thread for ROS2
    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    // Create an SFML window
    sf::RenderWindow window(sf::VideoMode(800, 600), "SuperMarket");

    while (window.isOpen())
    {
        // Process SFML events (e.g., close the window)
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Clear the window with a black color
        window.clear(sf::Color::Black);

        // Update window using ROS2 data
        // For example, let's draw a simple grid based on the occupancy grid data
        if (node->width_ > 0 && node->height_ > 0)
        {
            sf::RectangleShape cell(sf::Vector2f(10.0f, 10.0f)); // Each cell is a 5x5 pixel square
            for (uint32_t y = 0; y < node->height_; ++y)
            {
                for (uint32_t x = 0; x < node->width_; ++x)
                {
                    int8_t occupancy_value = node->map_data_[y * node->width_ + x];
                    if (occupancy_value == 0)  // Free space
                        cell.setFillColor(sf::Color::White);
                    else if (occupancy_value == 100)  // Occupied space
                        cell.setFillColor(sf::Color::Red);
                    else  // Unknown
                        cell.setFillColor(sf::Color::Black);

                    // Set position and draw the cell
                    cell.setPosition(x * 5.0f, y * 5.0f);
                    window.draw(cell);
                }
            }
        }

        // Display the window content
        window.display();
    }

    // Clean up ROS2 when SFML window is closed
    rclcpp::shutdown();
    ros_thread.join();

    return 0;
}