#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using namespace std::placeholders;

class PreApproach : public rclcpp::Node {
public:
    // Methods
    PreApproach() : rclcpp::Node::Node("pre_approach") {
        // Declare parameters
        this->declare_parameter("obstacle", 0.3);
        // Get parameter values from launch file
        this->get_params();
        // Create callable object fo laser callback
        auto laser_callable = std::bind(&PreApproach::laser_callback, this, _1);
        // Create a subscription for LaserScan messages
        _subscription = this->create_subscription<LaserScan>("/scan", 1, laser_callable);
        // Create a publisher for Twist messages
        _publisher = this->create_publisher<Twist>("/robot/cmd_vel", 1);
        RCLCPP_INFO(this->get_logger(), "Started pre_approach node sucessfully.");
    }
private:
    // Attributes
    rclcpp::Subscription<LaserScan>::SharedPtr _subscription;
    rclcpp::Publisher<Twist>::SharedPtr _publisher;
    float _obstacle;

    // Methods
    void get_params() {
        _obstacle = this->get_parameter("obstacle").get_parameter_value().get<float>();
    }
    void laser_callback(const LaserScan::SharedPtr msg) {
        float front_reading = msg->ranges[540];
        RCLCPP_DEBUG(this->get_logger(), "Front reading: %.2f.", front_reading);
        Twist vel_msg;
        if (front_reading > _obstacle) {
            vel_msg.linear.x = std::min(front_reading, float(1.0));
        } 
        else {
            vel_msg.linear.x = 0.0;
        }
        _publisher->publish(vel_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproach>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}