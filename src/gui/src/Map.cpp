/*
map.hpp IMPLEMETATION

This is the interface for the GUI that will be used to visualise the 
map and see the robot.

Subscribes to the /map topic that is provided by SLAM

Written: Daniel Monteiro
Date: 18/10/2024
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
    
    // Initialise odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", /* subscribe to topic /scan */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &Map::odom_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

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

    item_subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "id_detections",
        qos_settings,
        std::bind(                  
        &Map::item_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1));

    // create a subscriber for the battery state
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "battery_state",
        qos_settings,
        std::bind(                  
        &Map::battery_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1));

    // create the slam publisher
    slam_publisher_ = this->create_publisher<std_msgs::msg::Bool>("slam_request", 10);

    battery_percentage_ = 100.0f;
}

Map::~Map()
{
    RCLCPP_INFO(this->get_logger(), "Map Node has been terminated");
} 

void Map::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    // set the battery percentage 
    battery_percentage_ = msg->percentage;

    // RCLCPP_INFO(this->get_logger(), "Battery Voltage: %.2fV, Percentage: %.2f%%", 
    //             msg->voltage, msg->percentage);
}


void Map::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // break the message down into a quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  // do the math operation to convert the pose into a 3x3 matrix
  // for the orientation
  tf2::Matrix3x3 m(q);

  // extract the angles from the data
  m.getRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);

  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  current_pose_.z = msg->pose.pose.position.z;

//   RCLCPP_INFO(this->get_logger(), "Turtle: x: %f y: %f yaw: %f", current_pose_.x, 
//                                                                  current_pose_.x,
//                                                                  current_pose_.yaw);

}

void Map::item_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    // add the current item to the item logger
    std::string item;

    for(auto value: msg->data)
    {
        // if we've already processed this item then disregard it
        if(codes_seen_.find(value) != codes_seen_.end())
            continue;

        // add the item
        codes_seen_.insert(value);

        // determine the equivalent string based on mapping and add it to the item
        // logger
        if(ID_CODES_TO_ITEM.find(value) != ID_CODES_TO_ITEM.end())
        {
            item = ID_CODES_TO_ITEM.at(value);
            std::cout << "Adding one " << item << std::endl;
            item_logger_->add_item(item);
        }
        else
        {
            std::cout << "Unknown Item! " << std::endl;
            item_logger_->add_item("Unknown");
        }

    }

}

/*
Map callback receives a map message, stores the relative information 
and continues onwards
*/
void Map::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // extract relevant data information
    map_meta_data_.resolution = msg->info.resolution;
    map_meta_data_.width = msg->info.width;
    map_meta_data_.height = msg->info.height;
    map_data_ = msg->data;

    // process that we've read the current map
    new_map_available_ = true;
    
    // process the orientation of the map itself
    tf2::Quaternion q(
        msg->info.origin.orientation.x,
        msg->info.origin.orientation.y,
        msg->info.origin.orientation.z,
        msg->info.origin.orientation.w);


    current_map_pose_.x = msg->info.origin.orientation.x;
    current_map_pose_.y = msg->info.origin.orientation.y;
    current_map_pose_.z= msg->info.origin.orientation.z;


    // do the math operation to convert the pose into a 3x3 matrix
    // for the orientation
    tf2::Matrix3x3 m(q);

    // extract the angles from the data
    m.getRPY(current_map_pose_.roll, current_map_pose_.pitch, current_map_pose_.yaw);

    // transform the map based on the quaternion
    transform_map_orientation();

    // log the changes
    RCLCPP_INFO(this->get_logger(), "Grid! width: %d, height: %d, res: %f, ox: %f, oy: %f oxw: %f", msg->info.width, 
                                                                        msg->info.height,
                                                                        msg->info.resolution,
                                                                        msg->info.origin.orientation.x,
                                                                        msg->info.origin.orientation.y,
                                                                        msg->info.origin.orientation.w);

}

float Map::get_battery_percentage() const
{
    return battery_percentage_;
}

void Map::transform_map_orientation()
{   
    // resize the map
    transformed_map_.resize(map_data_.size());

    // set default values
    for(int i = 0;i < map_data_.size(); ++i)
        transformed_map_[i] = UNKNOWN;

    // loop through possible indexes and transform them
    double yaw = current_map_pose_.yaw;
    double cos_yaw = std::cos(-1*yaw);  // rotation
    double sin_yaw = std::sin(-1*yaw);

    // loop through each pixel and set it's transformed value
    for(int i = 0; i < map_data_.size(); ++i)
    {
        // get the x and y
        int x = i%static_cast<int>(map_meta_data_.width);
        int y = i/static_cast<int>(map_meta_data_.width);

        // Apply the inverse rotation
        int new_x = static_cast<int>(x * cos_yaw - y * sin_yaw);
        int new_y = static_cast<int>(x * sin_yaw + y * cos_yaw);

        transformed_map_[new_y * map_meta_data_.width + new_x] = map_data_[i];
    }
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

Map::Pose Map::get_current_pose() const
{ 
    return current_pose_;
}

Map::MapMetaData Map::get_map_meta_data() const
{
    return map_meta_data_;
}


std::shared_ptr<ItemLogger> Map::get_item_logger()
{
    return item_logger_;
}

bool Map::get_map_available() const
{
    return new_map_available_;
}


uint32_t Map::get_width() const
{
    return map_meta_data_.width;
}

uint32_t Map::get_height() const
{
    return map_meta_data_.height;
}

float Map::get_resolution() const
{
    return map_meta_data_.resolution;
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
