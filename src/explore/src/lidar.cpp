/*
lidar.cpp

This file contains the implementation of the lidar class

Written by William Ridley-Smith, 2024
*/

#include "explore/lidar.h"

using namespace std::chrono_literals;

// Constructor for a lidar node
Lidar::Lidar() : Node ("lidar_node")
{ 
    // Create subscriber to the lidar
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", /* subscribe to topic /scan */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &Lidar::scanCallback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

    // Initialise the stocktake time so that nothing is scanned immediately
    prev_stocktake_time_ = this->now();

    // Resize scan data member variables to account for current and 
    // previous scans
    scan_data_.resize(4);
    prev_scan_data_.resize(4);   


    // Create publisher for the intensity readings of the lidar
    scan_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>
                                                    ("lidar_intensity", 10);

    // Create publisher with a boolean to state if the robot should begin
    // a stocktake
    stocktake_pub_ = this->create_publisher<std_msgs::msg::Bool>
                                                ("spin_now", 10);

    // Create the timer that will control how often a stock take can be started
    // as well as how often the lidar is read
    update_timer_ = this->create_wall_timer(20ms, 
                                    std::bind(&Lidar::updateScanData, this));

    RCLCPP_INFO(this->get_logger(), 
                "Lidar_node has been successfully initialised");


}

// Destructor for lidar
Lidar::~Lidar()
{
    RCLCPP_INFO(this->get_logger(), "Lidar_node has been terminated");
}

// Callback for when lidar data is received
void Lidar::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    //save previous scan data into member variable
    prev_scan_data_ = scan_data_;

    // These will hold the sum of intensities within the angular range on 
    // each side
    long int left_sum_ = 0;
    long int right_sum_ = 0;

    // Calculate index range in the lidar readings which corresponds to the 
    // specified angular ranges
    int left_index_min_ = static_cast<int>((scan_left_[0] - msg->angle_min) / 
                                                    msg->angle_increment);
    int left_index_max_ = static_cast<int>((scan_left_[1] - msg->angle_min) / 
                                                    msg->angle_increment);
    int right_index_min_ = static_cast<int>((scan_right_[0] - msg->angle_min) / 
                                                    msg->angle_increment);
    int right_index_max_ = static_cast<int>((scan_right_[1] - msg->angle_min) / 
                                                    msg->angle_increment);


    // Calculate the average intensity of the left side in the angular range
    for(int i = left_index_min_; i < left_index_max_; i++) {
        left_sum_ += msg->intensities[i];
    }
    float left_intensity_ = left_sum_ / 
                            (left_index_max_ + 1 - left_index_min_);

    // Calculate the average intensity of the right side in the angular range
    for(int i = right_index_min_; i < right_index_max_; i++) {
        right_sum_ += msg->intensities[i];
    }
    float right_intensity_ = right_sum_ / 
                            (right_index_max_ + 1 - right_index_min_);

    // Saving the current scan data as well as the old scan data, so that 
    // spikes in intensity can be detected
    scan_data_[0] = prev_scan_data_[2]; // prev left intensity
    scan_data_[1] = prev_scan_data_[3]; // prev right intensity
    scan_data_[2] = left_intensity_;     // curr left intensity
    scan_data_[3] = right_intensity_;    // curr right intensity

    // Initialise spike detection to false
    is_intense_ = false;
    // To trigger a stocktake, a spike must occur and the intensity should be 
    // high enough
    // Respectively, these correspond to the new appearance of reflective tape 
    // and that this tape is close enough.
    if((scan_data_[2] > scan_data_[0]*change_threshold_ && 
                    left_intensity_ > intensity_threshold_) || 
        (scan_data_[3] > scan_data_[1]*change_threshold_ && 
                    right_intensity_ > intensity_threshold_)) {
        is_intense_ = true;
    }

}

// Updates the previously stored scan data and publishes new data
void Lidar::updateScanData()
{   
    // Create the message and insert the scan data into it
    auto intensity_message_ = std_msgs::msg::Float32MultiArray();
    intensity_message_.data.insert(intensity_message_.data.end(), 
                                    scan_data_.begin(), scan_data_.end());
    scan_data_pub_->publish(intensity_message_);

    // Create the stocktake message and insert the data
    std_msgs::msg::Bool spin_message_;
    spin_message_.data = is_intense_;

    // Get current time
    rclcpp::Time time_now_ = this->now();


    // Sufficient time must have passed, and the intensity conditions are passed
    if ((time_now_ - prev_stocktake_time_).nanoseconds() / 1e9
         > stocktake_frequency_ && is_intense_) {
        prev_stocktake_time_ = this->now();
        stocktake_pub_->publish(spin_message_);
    }
    // Otherwise the message defaults to false
    else {
        spin_message_.data = false;
        stocktake_pub_->publish(spin_message_);
    }
}

// Create deep copy of scan data to not change original data
std::vector<double> Lidar::getScanData()
{   
    std::vector<double> deep_copy_scan_data_ = scan_data_;
    return deep_copy_scan_data_;
}


#ifdef LIDAR_MAIN

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lidar>());
    rclcpp::shutdown();
}
#endif