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

    //resize scan data member variables
    scan_data_.resize(3);
    prev_scan_data_.resize(3);   


    // initialise publisher
    scan_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("lidar", 100);
    // create the timer that will cotrol how often the state gets published
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

    for (int num = 0; num < 3; num++) {
        if (std::isinf(msg->intensities.at(scan_angle[num]))) {
        scan_data_[num] = 0;
        } else {
        scan_data_[num] = msg->intensities.at(scan_angle[num]);
        }
    }
}


void Lidar::update_scan_data()
{   
    // create the message and insert the scan data into it
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.insert(message.data.end(), scan_data_.begin(), scan_data_.end());

    // publish the message
    scan_data_pub_->publish(message);
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