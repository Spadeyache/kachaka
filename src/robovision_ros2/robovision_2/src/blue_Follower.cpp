#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int32.hpp>

class BlueFollowerNode : public rclcpp::Node
{
public:
    BlueFollowerNode() : Node("blue_follower")
    {
        // Subscriber
        delta_x_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/kn_delta", 10, std::bind(&BlueFollowerNode::callback_delta_x, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Blue Follower Node started...");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr delta_x_subscriber_;

    void callback_delta_x(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int delta_x = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received delta x: %d", delta_x);
        // Add logic to handle delta_x here
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlueFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

