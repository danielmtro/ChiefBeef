#include "explore/lidar.h"

using namespace std::chrono_literals;

//--Lidar Implementation--------------------------------------------
Lidar::Lidar() : Node ("lidar_node")
{ 
    //create LiDar subscriber
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", /* subscribe to topic /scan */ \
        rclcpp::SensorDataQoS(), /* use the qos number set by rclcpp */ \
        std::bind(                  
        &Lidar::scan_callback, /* bind the callback function */ \
        this, \
        std::placeholders::_1)
        );

    prev_stocktake_time_ = this->now();

    //resize scan data member variables to account for curr and prev intensities
    scan_data_.resize(4);
    prev_scan_data_.resize(4);   


    // initialise publishers
    scan_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("lidar_intensity", 10);
    stocktake_pub_ = this->create_publisher<std_msgs::msg::Bool>("spin_now", 10);
    // create the timer that will cotrol how often a stock take can be started
    update_timer_ = this->create_wall_timer(20ms, std::bind(&Lidar::update_scan_data, this));

    RCLCPP_INFO(this->get_logger(), "Lidar_node has been successfully initialised");


}

//--
Lidar::~Lidar()
{
    RCLCPP_INFO(this->get_logger(), "Lidar_node has been terminated");
}

//--
void Lidar::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    //save previous scan data into member variable
    prev_scan_data_ = scan_data_;

    // These will hold the sum of intensities within the angular range on each side
    long int left_sum = 0;
    long int right_sum = 0;

    // Calculate index range corresponding to the specified angular ranges
    int left_index_min = static_cast<int>((scan_left[0] - msg->angle_min) / msg->angle_increment);
    int left_index_max = static_cast<int>((scan_left[1] - msg->angle_min) / msg->angle_increment);
    int right_index_min = static_cast<int>((scan_right[0] - msg->angle_min) / msg->angle_increment);
    int right_index_max = static_cast<int>((scan_right[1] - msg->angle_min) / msg->angle_increment);


    // Calculate the average intensity of each side
    for(int i = left_index_min; i < left_index_max; i++) {
        left_sum += msg->intensities[i];
    }
    float left_intensity = left_sum / (left_index_max + 1 - left_index_min);

    for(int i = right_index_min; i < right_index_max; i++) {
        right_sum += msg->intensities[i];
    }
    float right_intensity = right_sum / (right_index_max + 1 - right_index_min);

    scan_data_[0] = prev_scan_data_[2]; // prev left intensity
    scan_data_[1] = prev_scan_data_[3]; // prev right intensity
    scan_data_[2] = left_intensity;
    scan_data_[3] = right_intensity;

    is_intense = false;
    // To trigger a stocktake, a spike must occur and the intensity should be high enough
    // Respectively, these correspond to the new appearance of reflective tape and that this tape is close enough.
    if((scan_data_[2] > scan_data_[0]*change_threshold && left_intensity > intensity_threshold) || (scan_data_[3] > scan_data_[1]*change_threshold && right_intensity > intensity_threshold)) {
        is_intense = true;
    }

}


void Lidar::update_scan_data()
{   
    // create the message and insert the scan data into it
    auto intensity_message = std_msgs::msg::Float32MultiArray();
    intensity_message.data.insert(intensity_message.data.end(), scan_data_.begin(), scan_data_.end());
    scan_data_pub_->publish(intensity_message);

    // Create the stocktake message and insert the data
    std_msgs::msg::Bool spin_message;
    spin_message.data = is_intense;

    rclcpp::Time time_now = this->now();

    // Sufficient time must have passed, and the intensity conditions are passed
    if ((time_now - prev_stocktake_time_).nanoseconds() / 1e9 > stocktake_frequency && is_intense) {
        prev_stocktake_time_ = this->now();
        stocktake_pub_->publish(spin_message);
    }
    // Otherwise the message defaults to false
    else {
        spin_message.data = false;
        stocktake_pub_->publish(spin_message);
    }
}


std::vector<double> Lidar::get_scan_data()
{   
    // create a deep copy as to not change the original data (getter)
    std::vector<double> deep_copy_scan_data = scan_data_;
    return deep_copy_scan_data;
}


#ifdef LIDAR_MAIN

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lidar>());
    rclcpp::shutdown();
}
#endif