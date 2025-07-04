#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotorPublisher : public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    public:
    MotorPublisher() : Node("motor_publisher"){
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/tid_kachaka/manual_control/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TwistPublisher::timer_callback, this));
    }

    private:
        void timer_callback(){
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f", message.linear.x, message.angular.z);
        }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}