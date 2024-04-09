#include "my_components/attach_server.hpp"

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options) : Node("approach_service_server", options) {
    // Create a subscription for LaserScan messages
    auto laser_callable = std::bind(&AttachServer::laser_callback, this, _1);
    _laser_subscription = this->create_subscription<LaserScan>("/scan", 1, laser_callable);
    // Create the service server
    auto callable_handle = std::bind(&AttachServer::handle_service, this, _1, _2);
    _service = this->create_service<GoToLoading>("approach_shelf", callable_handle);
    // Initialize transform buffer and listener
    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    // Intizalize static transform broadcaster
    _tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Create a publisher for Twist anc Empty messages
    _publisher = this->create_publisher<Twist>("/robot/cmd_vel", 1);
    _elevator_publisher = this->create_publisher<Empty>("/elevator_up", 1);
    // Inform server creation
    RCLCPP_INFO(this->get_logger(), "Service server started.");
}

void AttachServer::handle_service(const std::shared_ptr<GoToLoading::Request> request, std::shared_ptr<GoToLoading::Response> response) {
    // Handle request
    RCLCPP_INFO(this->get_logger(), "Legs detected: %i.", _num_legs);
    if (_num_legs == 2 && request->attach_to_shelf) {
        // Publish cart frame
        RCLCPP_INFO(this->get_logger(), "Publishing cart frame");
        this->publish_cart_frame();
        // Handle approach
        RCLCPP_INFO(this->get_logger(), "True request");
        RCLCPP_INFO(this->get_logger(), "Approaching to cart.");    
        _approach_timer = this->create_wall_timer(100ms, std::bind(&AttachServer::approach_cart, this));
        response->complete = true;
    }
    else if (_num_legs == 2 && !request->attach_to_shelf){
        // Publish cart frame
        RCLCPP_INFO(this->get_logger(), "Publishing cart frame");
        this->publish_cart_frame();
        // Handle false request
        RCLCPP_INFO(this->get_logger(), "False request.");
        RCLCPP_WARN(this->get_logger(), "The final approach is ommited.");
        response->complete = false;
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Not enough legs, returning false.");
        response->complete = false;
    }   
}

void AttachServer::laser_callback(const LaserScan::SharedPtr msg) {
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

void AttachServer::publish_cart_frame() {
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

    RCLCPP_DEBUG(this->get_logger(), "X coordinate from laser: %.2f.", x);
    RCLCPP_DEBUG(this->get_logger(), "Y coordinate from laser: %.2f.", y);

    // Transform coordinates from laser to odom
    std::string child_frame = "robot_front_laser_base_link";
    std::string parent_fame = "odom";
    geometry_msgs::msg::TransformStamped t;
    // Look up for the transformation between odom and laser frames
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
        RCLCPP_DEBUG(this->get_logger(), "Transformed X coordinate in odom: %.2f.", odom_point.point.x);
        RCLCPP_DEBUG(this->get_logger(), "Transformed Y coordinate in odom: %.2f.", odom_point.point.y);
        RCLCPP_DEBUG(this->get_logger(), "Transformed Z coordinate in odom: %.2f.", odom_point.point.z);
        // Create new frame
        geometry_msgs::msg::TransformStamped cart_frame;
        cart_frame = t;
        cart_frame.child_frame_id = "cart_frame";
        cart_frame.transform.translation.x = odom_point.point.x;
        cart_frame.transform.translation.y = odom_point.point.y;
        // Broadcast the new frame
        _tf_static_broadcaster->sendTransform(cart_frame);
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform from robot_front_laser_base_link to odom");
        RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    }
}

void AttachServer::approach_cart() {
    geometry_msgs::msg::TransformStamped t;
    t = _tf_buffer->lookupTransform("robot_base_link", "cart_frame", tf2::TimePointZero);
    // Compute velocities
    auto x = t.transform.translation.x + 0.5;
    auto y = t.transform.translation.y;

    float error_distance = std::sqrt(x*x + y*y);
    float error_yaw = std::atan2(y, x);
    RCLCPP_DEBUG(this->get_logger(), "Error distance: %.4f", error_distance);
    RCLCPP_DEBUG(this->get_logger(), "Error yaw: %.4f", error_yaw);
    Twist vel_msg;
    if (error_distance > 0.01) {
        vel_msg.angular.z = -0.5 * error_yaw;  
        vel_msg.linear.x = std::min(1.0 * error_distance, 0.75);
        RCLCPP_DEBUG(this->get_logger(), "Z angular: %.3f", vel_msg.angular.z);
        RCLCPP_DEBUG(this->get_logger(), "X linear: %.3f", vel_msg.linear.x);
    }
    else {
        vel_msg.angular.z = 0;
        vel_msg.linear.x = 0;
        RCLCPP_INFO(this->get_logger(), "Final approach completed.");
        _approach_timer->cancel();
        RCLCPP_INFO(this->get_logger(), "Lifting the shelf.");
        Empty msg;
        _elevator_publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Process finished.");
    }
    // Pubish velocity
    _publisher->publish(vel_msg);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)