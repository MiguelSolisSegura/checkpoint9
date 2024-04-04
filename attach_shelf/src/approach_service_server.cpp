#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include <functional>
#include <memory>
#include "sensor_msgs/msg/laser_scan.hpp"

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
        RCLCPP_INFO(this->get_logger(), "Service server started.");
    }
private:
    // Attributes
    rclcpp::Service<GoToLoading>::SharedPtr _service;
    rclcpp::Subscription<LaserScan>::SharedPtr _laser_subscription;
    bool _high_intensity = false;
    // Methods
    void handle_service(const std::shared_ptr<GoToLoading::Request> request, std::shared_ptr<GoToLoading::Response> response) {
        if (request->attach_to_shelf) {
            RCLCPP_INFO(this->get_logger(), "TRUE request");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "FALSE request");
        }
        response->complete = _high_intensity;
    }
    void laser_callback(const LaserScan::SharedPtr msg) {
        for (int i = 0; i < int(msg->intensities.size()); i++) {
            if (msg->intensities[i] > 100.0) {
                RCLCPP_INFO(this->get_logger(), "High reading at: %i.", i);
                _high_intensity = true;
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachShelf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}