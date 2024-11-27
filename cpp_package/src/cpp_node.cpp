#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"  // For handling image messages
#include "std_msgs/msg/string.hpp"    // For handling mode messages (greyscale or color)
#include "cv_bridge/cv_bridge.h"      // For converting OpenCV images to ROS messages
#include "opencv2/opencv.hpp"         // OpenCV for image processing

using std::placeholders::_1;

class ImageProcessorNode : public rclcpp::Node
{
public:
    ImageProcessorNode()
    : Node("image_processor_node"), current_mode_set(false)
    {
        // Subscriber to listen for raw color images
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/image_raw", 10, std::bind(&ImageProcessorNode::image_callback, this, _1)
        );

        // Publisher to send processed images to /camera1/image_out
        image_out_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera1/image_out", 10
        );

        // Subscriber to listen for mode change messages
        mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/mode", 10, std::bind(&ImageProcessorNode::mode_callback, this, _1)
        );
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        // Wait for the mode to be selected before processing the image
        if (!current_mode_set) {
            RCLCPP_WARN(this->get_logger(), "Mode not set yet. Waiting for mode...");
            return;  // Don't process the image until the mode is set
        }

        // Convert the sensor_msgs::msg::Image to OpenCV format
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image; // Assuming image is in BGR8 format

        if (current_mode) {
            // If in Greyscale mode, convert to grayscale
            cv::Mat gray_img;
            cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

            // Convert the grayscale image back to ROS message format
            sensor_msgs::msg::Image::SharedPtr gray_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "mono8", gray_img
            ).toImageMsg();

            // Publish the grayscale image to the /camera1/image_out topic
            image_out_publisher_->publish(*gray_msg);
            RCLCPP_INFO(this->get_logger(), "Mode is Greyscale. Published grayscale image to /camera1/image_out");
        } else {
            // If in Color mode, simply publish the original image to the /camera1/image_out topic
            sensor_msgs::msg::Image::SharedPtr color_msg = msg;
            image_out_publisher_->publish(*color_msg);
            RCLCPP_INFO(this->get_logger(), "Mode is Color. Published color image to /camera1/image_out");
        }

        // Log the received image dimensions
        RCLCPP_INFO(this->get_logger(), "Received image with width: %d and height: %d", msg->width, msg->height);
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Update the mode based on the received message
        if (msg->data == "Mode 1: Greyscale") {
            current_mode = true;  // Set mode to Greyscale
            current_mode_set = true; // Indicate that the mode is now set
            RCLCPP_INFO(this->get_logger(), "Mode switched to Greyscale");
        } else if (msg->data == "Mode 2: Color") {
            current_mode = false;  // Set mode to Color
            current_mode_set = true; // Indicate that the mode is now set
            RCLCPP_INFO(this->get_logger(), "Mode switched to Color");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_out_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;

    bool current_mode;  // true for Greyscale, false for Color
    bool current_mode_set; // Flag to check if the mode has been set
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

