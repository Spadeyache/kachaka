#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ImageProcessingNode : public rclcpp::Node
{
public:
    ImageProcessingNode() : Node("image_processing"), is_image_(false)
    {
        counter_ = 0;

        //Subscribers
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/er_kachaka/front_camera/image_raw", qos, std::bind(&ImageProcessingNode::callback_image, this, std::placeholders::_1));

        
        //Processing
        image_processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ImageProcessingNode::image_processing, this));

        RCLCPP_INFO(this->get_logger(), "Starting image_processing application in cpp...");
    }

private:

    bool is_image_;
    int counter_;
    cv::Mat image_;
    sensor_msgs::msg::Image::SharedPtr image_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::TimerBase::SharedPtr image_processing_timer_;
    
    void callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        is_image_ = true;
    }

    void image_processing()
    {
        if (is_image_){
            if (counter_ == 0){
                std::cout << "size: rows: " << image_.rows << 
                             ", cols: " << image_.cols << 
                             ", depth: " << image_.channels() << std::endl;
            }
            counter_++;

            // Create a binary mask where blue channel > 200
            cv::Mat binary_mask;
            cv::Mat hsv;
            cv::cvtColor(image_, hsv, cv::COLOR_BGR2HSV);
            // cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255), binary_mask);
            cv::inRange(hsv, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), binary_mask);

            // Display the binary mask
            cv::imshow("binary_mask", binary_mask);
            cv::imshow("raw", image_);
            cv::waitKey(1);
   
        }

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
