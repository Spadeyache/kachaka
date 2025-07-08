// I dont use this for kachaka
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MotorPublisher : public rclcpp::Node {
public:
    MotorPublisher() : Node("motor_publisher"), state_(0)
    {
        // Timer 1: velocity publisher every 100ms
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/er_kachaka/manual_control/cmd_vel", 10);
        motion_timer_ = this->create_wall_timer(100ms, std::bind(&MotorPublisher::motion_callback, this));

        // Timer 2: info logger every 1s
        info_timer_ = this->create_wall_timer(1.5s, [this]() {
            RCLCPP_INFO(this->get_logger(), "nodegit _loop");
            state_ += 1;
        });
    }

private:
    void motion_callback()
    {
        if(state_ % 2 == 0){
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 0.0;
            message.angular.z = 1.0;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f", message.linear.x, message.angular.z);
        }
        else{
            auto message = geometry_msgs::msg::Twist();
        message.linear.x = -3.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f", message.linear.x, message.angular.z);
        }
        

    }
    int state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
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
