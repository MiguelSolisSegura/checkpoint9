#include "my_components/attach_client.hpp"

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options) : rclcpp::Node::Node("approach_service_client", options) {
    // Declare parameters
    this->declare_parameter("final_approach", true);
    // Get parameter values from launch file
    this->get_params();
    // Service client
    _client = this->create_client<GoToLoading>("approach_shelf");
    while (!_client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service /approach_shelf to become available.");
    }
    // Send request to service server
    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = _final_approach;
    auto callable_response = std::bind(&AttachClient::handle_response, this, _1);
    RCLCPP_INFO(this->get_logger(), "Sending request to service server.");
    _client->async_send_request(request, callable_response);

    RCLCPP_INFO(this->get_logger(), "Starting final approach.");
}

void AttachClient::get_params() {
    _final_approach = this->get_parameter("final_approach").get_parameter_value().get<bool>();
}

void AttachClient::handle_response(const rclcpp::Client<GoToLoading>::SharedFuture future) {
    auto response = future.get();
    if (response->complete) {
        RCLCPP_INFO(this->get_logger(), "Two legs detected, final approach performed.");
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Final approach not performed.");
    }
    rclcpp::shutdown();
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)