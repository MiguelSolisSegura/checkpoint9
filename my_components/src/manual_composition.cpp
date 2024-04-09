#include "my_components/pre_approach.hpp"
#include "my_components/attach_server.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/utilities.hpp"
#include <cstddef>
#include <cstdio>
#include <memory>

int main(int argc, char ** argv) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto pre_approach_node = std::make_shared<my_components::PreApproach>(options);
    exec.add_node(pre_approach_node);
    auto attach_server_node = std::make_shared<my_components::AttachServer>(options);
    exec.add_node(attach_server_node);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}