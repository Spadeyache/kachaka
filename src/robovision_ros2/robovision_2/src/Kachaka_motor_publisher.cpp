#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class MotorPublisher : public rclcpp::Node {
public:
    MotorPublisher() : Node("motor_publisher"), state_(0)
    {
        // Timer 1: velocity publisher every 100ms
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/er_kachaka/manual_control/cmd_vel", 10);
        motion_timer_ = this->create_wall_timer(100ms, std::bind(&MotorPublisher::motion_callback, this));

        // Subscriber to delta_x
        delta_x_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/kn_delta", 10, std::bind(&MotorPublisher::callback_delta_x, this, std::placeholders::_1));

        // Timer 2: info logger every 1s
        info_timer_ = this->create_wall_timer(1.5s, [this]() {
            RCLCPP_INFO(this->get_logger(), "nodegit _loop");
            state_ += 1;
        });
    }

private:
    void motion_callback()
    {
        // Instead of simple on-off control, we can do sth like using the past delta_x to predict the next delta_x
        // and then use the predicted delta_x to control the motor
        // it would be better if we can use pid control to make the motor move more smoothly
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;
        if(delta_x_ > 300){
            message.angular.z = -1.0;
        }
        else if(delta_x_ < -300){
            message.angular.z = 1.0;
        }
        else if(abs(delta_x_) < 120){
            message.angular.z = 0.0;
            message.linear.x = 0.3;
            // add a code for the motor to move till 10cm form the object
        }
        else{
            if (delta_x_ > 0) {
                message.angular.z = -0.1;
            } else {
                message.angular.z = 0.1;
            }
        }
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: angular.z=%.2f", message.angular.z);
    }

    void callback_delta_x(const std_msgs::msg::Int32::SharedPtr msg)
    {
        delta_x_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received delta x: %d", delta_x_);
    }

    int state_;
    int delta_x_ = 0;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr delta_x_subscriber_;
    rclcpp::TimerBase::SharedPtr motion_timer_;
    rclcpp::TimerBase::SharedPtr info_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
