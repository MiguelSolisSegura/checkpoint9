#ifndef COMPOSITION_ATTACH_SERVER_HPP_
#define COMPOSITION_ATTACH_SERVER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "custom_attach_interface/srv/go_to_loading.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include <future>
#include "my_components/visibility_control.h"

using GoToLoading = custom_attach_interface::srv::GoToLoading;
using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using String = std_msgs::msg::String;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
    COMPOSITION_PUBLIC
    // Constructor
    explicit AttachServer(const rclcpp::NodeOptions &options);
private:
    // General attributes
    rclcpp::Service<GoToLoading>::SharedPtr _service;
    rclcpp::Subscription<LaserScan>::SharedPtr _laser_subscription;
    rclcpp::Publisher<Twist>::SharedPtr _publisher;
    rclcpp::Publisher<String>::SharedPtr _elevator_publisher;
    // Scanning attributes
    int _num_legs = 0;
    int _total_readings;
    std::vector<std::pair<int, float>> _leg_data;
    // Transform attributes
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_static_broadcaster;
    // Final approach attributes
    rclcpp::TimerBase::SharedPtr _approach_timer;
    // Methods
    void handle_service(const std::shared_ptr<GoToLoading::Request> request, std::shared_ptr<GoToLoading::Response> response);
    void laser_callback(const LaserScan::SharedPtr msg);
    void publish_cart_frame();
    void approach_cart();
};

}

#endif // COMPOSITION_ATTACH_SERVER_HPP_