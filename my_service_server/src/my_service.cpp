#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"  // To publish the mode (Greyscale/Color)

#include <memory>

// Publisher to send mode messages
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

// Variable to store the current mode (true = Greyscale, false = Color)
bool current_mode = false; // Default to Color mode

void change_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        current_mode = true;  // Set mode to Greyscale
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode changed to Greyscale");
        response->success = true;
        response->message = "Mode set to Greyscale";

        // Publish the mode as a message
        std_msgs::msg::String mode_msg;
        mode_msg.data = "Mode 1: Greyscale";
        publisher->publish(mode_msg);
    } else {
        current_mode = false;  // Set mode to Color
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode changed to Color");
        response->success = true;
        response->message = "Mode set to Color";

        // Publish the mode as a message
        std_msgs::msg::String mode_msg;
        mode_msg.data = "Mode 2: Color";
        publisher->publish(mode_msg);
    }

    // Log the response message
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%s]", response->message.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = rclcpp::Node::make_shared("mode_service_node");

    // Create the service to change the mode
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service =
        node->create_service<std_srvs::srv::SetBool>("change_mode", &change_mode);

    // Create the publisher for the mode messages (published to the "mode" topic)
    publisher = node->create_publisher<std_msgs::msg::String>("mode", 10);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode service is ready.");

    // Spin the node to process incoming requests
    rclcpp::spin(node);

    // Shutdown after spinning
    rclcpp::shutdown();
}

