#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include "custom_attach_interface/srv/go_to_loading.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using GoToLoading = custom_attach_interface::srv::GoToLoading;
using namespace std::placeholders;
using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
    // Methods
    PreApproach() : rclcpp::Node::Node("pre_approach") {
        // Declare parameters
        this->declare_parameter("obstacle", 0.3);
        this->declare_parameter("degrees", 90);
        this->declare_parameter("final_approach", false);
        // Get parameter values from launch file
        this->get_params();
        // Create a subscription for LaserScan and Odometry messages
        auto laser_callable = std::bind(&PreApproach::laser_callback, this, _1);
        _laser_subscription = this->create_subscription<LaserScan>("/scan", 1, laser_callable);
        auto odometry_callable = std::bind(&PreApproach::odometry_callback, this, _1);
        _odomery_subscription = this->create_subscription<Odometry>("/odom", 1, odometry_callable);
        // Create a publisher for Twist messages
        _publisher = this->create_publisher<Twist>("/robot/cmd_vel", 1);
        // Logic for rotation
        _enable_rotation = false;
        _total_rotation = 0;
        // Service client
        _client = this->create_client<GoToLoading>("approach_shelf");
        while (!_client->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service /approach_shelf to become available.");
        }

        RCLCPP_INFO(this->get_logger(), "Starting forward motion.");
    }
private:
    // Attributes
    rclcpp::Subscription<LaserScan>::SharedPtr _laser_subscription;
    rclcpp::Subscription<Odometry>::SharedPtr _odomery_subscription;
    rclcpp::Publisher<Twist>::SharedPtr _publisher;
    rclcpp::Client<GoToLoading>::SharedPtr _client;
    float _obstacle;
    float _degrees;
    bool _final_approach;
    float _last_rotation;
    float _total_rotation;
    bool _enable_rotation;

    // Methods
    void get_params() {
        _obstacle = this->get_parameter("obstacle").get_parameter_value().get<float>();
        _degrees = this->get_parameter("degrees").get_parameter_value().get<int>() * M_PI / 180;
        _final_approach = this->get_parameter("final_approach").get_parameter_value().get<bool>();
    }
    void laser_callback(const LaserScan::SharedPtr msg) {
        Twist vel_msg;
        float front_reading = msg->ranges[540];
        RCLCPP_DEBUG(this->get_logger(), "Front reading: %.2f.", front_reading);
        if (front_reading > _obstacle) {
            vel_msg.linear.x = std::min(front_reading, float(1.0));
        } 
        else {
            vel_msg.linear.x = 0.0;
            RCLCPP_INFO(this->get_logger(), "Starting rotation.");
            _laser_subscription = nullptr;
            _enable_rotation = true;         
        }
        _publisher->publish(vel_msg);
        
    }
    void odometry_callback(const Odometry::SharedPtr msg) {
        if (!_enable_rotation) {
            _last_rotation = 2 * std::atan2(msg->pose.pose.orientation.w, msg->pose.pose.orientation.z);
        }
        else {
            float current_rotation = 2 * std::atan2(msg->pose.pose.orientation.w, msg->pose.pose.orientation.z);
            float rotation_delta =  std::abs(current_rotation - _last_rotation);
            _total_rotation += rotation_delta;
            _last_rotation = current_rotation;
            Twist vel_msg;
            float rotation_error = std::signbit(_degrees) ? _degrees + _total_rotation : _degrees - _total_rotation;
            RCLCPP_DEBUG(this->get_logger(), "Rotation error: %.2f.", rotation_error);
            vel_msg.angular.z = 2 * (rotation_error);
            if (std::abs(rotation_error) <= (M_PI / 180)) {
                RCLCPP_INFO(this->get_logger(), "Rotation finished.");
                _odomery_subscription = nullptr;
                vel_msg.angular.z = 0.0;
                auto request = std::make_shared<GoToLoading::Request>();
                request->attach_to_shelf = _final_approach;
                auto callable_response = std::bind(&PreApproach::handle_response, this, _1);
                RCLCPP_INFO(this->get_logger(), "Sending request to service server.");
                _client->async_send_request(request, callable_response);
                
            }
            _publisher->publish(vel_msg);
        }
    }
    void handle_response(const rclcpp::Client<GoToLoading>::SharedFuture future) {
        auto response = future.get();
        if (response->complete) {
            RCLCPP_INFO(this->get_logger(), "Two legs detected, final approach performed.");
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Final approach not performed.");
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproach>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}