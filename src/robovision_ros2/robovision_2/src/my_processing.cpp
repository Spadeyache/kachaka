#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int32.hpp>


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

        // Publishers
        delta_x_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/kn_delta", 10);

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
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr delta_x_publisher_;
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

            // Reduce resolution for less computation
            cv::Mat resized_image;
            cv::resize(image_, resized_image, cv::Size(), 0.5, 0.5);

            // Define ROI to ignore the top third of the image
            int roi_start_y = resized_image.rows / 5;
            cv::Rect roi(0, roi_start_y, resized_image.cols, resized_image.rows - roi_start_y);
            cv::Mat image_roi = resized_image(roi);

            // Convert to HSV and create a binary mask
            cv::Mat hsv;
            cv::cvtColor(image_roi, hsv, cv::COLOR_BGR2HSV);
            cv::Mat binary_mask;
            cv::inRange(hsv, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), binary_mask);

            // Find the center of intensity
            cv::Moments m = cv::moments(binary_mask, true);
            int center_x = static_cast<int>(m.m10 / m.m00);
            int center_y = static_cast<int>(m.m01 / m.m00);

            // Adjust center_y to account for the ROI offset
            center_y += roi_start_y;

            // Calculate delta x
            int image_center_x = resized_image.cols / 2;
            int delta_x = center_x - image_center_x;

            // Publish delta x
            auto message = std_msgs::msg::Int32();
            message.data = delta_x;
            delta_x_publisher_->publish(message);

            // Draw a red dot at the center of intensity
            cv::circle(resized_image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), -1);

            // Display the binary mask and the image with the red dot
            cv::imshow("binary_mask", binary_mask);
            cv::imshow("raw", resized_image);
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
