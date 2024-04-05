#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/static_transform_broadcaster.h"


using GoToLoading = attach_shelf::srv::GoToLoading;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::placeholders;

class ApproachShelf : public rclcpp::Node {
public:
    // Methods
    ApproachShelf() : Node("approach_service_server") {
        // Create a subscription for LaserScan messages
        auto laser_callable = std::bind(&ApproachShelf::laser_callback, this, _1);
        _laser_subscription = this->create_subscription<LaserScan>("/scan", 1, laser_callable);
        // Create the service server
        auto callable_handle = std::bind(&ApproachShelf::handle_service, this, _1, _2);
        _service = this->create_service<GoToLoading>("approach_shelf", callable_handle);
        // Initialize transform buffer and listener
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
        // Intizalize static transform broadcaster
        _tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // Inform server creation
        RCLCPP_INFO(this->get_logger(), "Service server started.");
    }
private:
    // General attributes
    rclcpp::Service<GoToLoading>::SharedPtr _service;
    rclcpp::Subscription<LaserScan>::SharedPtr _laser_subscription;

    // Scanning attributes
    int _num_legs = 0;
    int _total_readings;
    std::vector<std::pair<int, float>> _leg_data;

    // Transform attributes
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_static_broadcaster;

    // Methods
    void handle_service(const std::shared_ptr<GoToLoading::Request> request, std::shared_ptr<GoToLoading::Response> response) {
        if (request->attach_to_shelf) {
            RCLCPP_INFO(this->get_logger(), "TRUE request");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "FALSE request");
        }
        RCLCPP_INFO(this->get_logger(), "Legs detected: %i.", _num_legs);
        if (_num_legs == 2) {
            // A: Robot to left leg
            float a = _leg_data[0].second;
            // B: Robot to right leg
            float b = _leg_data[1].second;
            // Theta: Angle between legs
            float theta = 2 * M_PI * std::abs(_leg_data[0].first - _leg_data[1].first) / _total_readings;
            // C: Right to left leg (Law of Cosines)
            float c = std::sqrt(a*a + b*b - 2*a*b * std::cos(theta));
            // D: Distance from robot to the middle of C (Apollonius Theorem)
            float d = 0.5 * std::sqrt(2*(a*a + b*b) - c*c);
            // Alpha: Angle for vertex BC (Law of Sines)
            float alpha = std::asin(a * std::sin(theta) / c);
            // Beta: Angle for vertex BD (Law of Sines)
            float beta = std::asin((c/2) * std::sin(alpha) / d);
            // X: Coordinate x from laser link
            float x = d * std::sin(alpha + beta);
            // Y: Coordinate y from laser link
            float y = d * std::cos(alpha + beta);


            RCLCPP_INFO(this->get_logger(), "X coordinate: %.2f.", x);
            RCLCPP_INFO(this->get_logger(), "Y coordinate: %.2f.", y);

            // Transform coordinates from laser to odom
            std::string child_frame = "robot_front_laser_link";
            std::string parent_fame = "robot_odom";
            geometry_msgs::msg::TransformStamped t;
            // Look up for the transformation between frames
            try {
                auto tf_time = tf2::TimePointZero;
                t = _tf_buffer->lookupTransform(parent_fame, child_frame, tf_time);
                // Create a PointStamped for the point (x, y)
                geometry_msgs::msg::PointStamped laser_point;
                laser_point.point.x = x;
                laser_point.point.y = y;
                laser_point.header.frame_id = child_frame;
                laser_point.header.stamp = t.header.stamp;
                // Transform the point from laser_link to odom
                geometry_msgs::msg::PointStamped odom_point;
                tf2::doTransform(laser_point, odom_point, t);
                // Now odom_point contains the (x, y) coordinates in the "odom" frame
                RCLCPP_INFO(this->get_logger(), "Transformed X coordinate in odom: %.2f.", odom_point.point.x);
                RCLCPP_INFO(this->get_logger(), "Transformed Y coordinate in odom: %.2f.", odom_point.point.y);
                RCLCPP_INFO(this->get_logger(), "Transformed Z coordinate in odom: %.2f.", odom_point.point.z);
                // Create new frame
                geometry_msgs::msg::TransformStamped cart_frame;
                cart_frame = t;
                cart_frame.child_frame_id = "cart_frame";
                cart_frame.transform.translation.x = odom_point.point.x;
                cart_frame.transform.translation.y = odom_point.point.y;
                _tf_static_broadcaster->sendTransform(cart_frame);
            } 
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform from robot_front_laser_link to robot_odom");
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            return;
            }
        }
        response->complete = _num_legs == 2 ? true : false;
    }

    void laser_callback(const LaserScan::SharedPtr msg) {
        int legs = 0;
        _total_readings = int(msg->ranges.size());
        _leg_data.clear();
        bool leg_flag = false;
        int leg_start;
        for (int i = 0; i < int(msg->intensities.size()); i++) {
            if (msg->intensities[i] > 100.0) {
                RCLCPP_DEBUG(this->get_logger(), "High reading at: %i.", i);
                if (!leg_flag) {
                    leg_flag = true;
                    leg_start = i;
                }
            }
            else {
                if (leg_flag) {
                    leg_flag = false;
                    int leg_center = (leg_start + i) / 2;
                    RCLCPP_DEBUG(this->get_logger(), "Leg center at: %i.", leg_center);
                    legs += 1;
                    _leg_data.push_back({leg_center, msg->ranges[leg_center]});
                }
            }
        }
        _num_legs = legs;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachShelf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}