#ifndef COMPOSITION_PRE_APPROACH_COMPONENT_HPP_
#define COMPOSITION_PRE_APPROACH_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using namespace std::placeholders;

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
    // Macros
    COMPOSITION_PUBLIC
    // Methods
    explicit PreApproach(const rclcpp::NodeOptions &options);
private:
    // Attributes
    rclcpp::Subscription<LaserScan>::SharedPtr _laser_subscription;
    rclcpp::Subscription<Odometry>::SharedPtr _odomery_subscription;
    rclcpp::Publisher<Twist>::SharedPtr _publisher;
    float _obstacle;
    float _degrees;
    float _last_rotation;
    float _total_rotation;
    bool _enable_rotation;
    // Methods
    void get_params();
    void laser_callback(const LaserScan::SharedPtr msg);
    void odometry_callback(const Odometry::SharedPtr msg);
};

}

#endif // COMPOSITION_PRE_APPROACH_COMPONENT_HPP_