#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <iostream>
#include <vector>
#include <exception>

class PointCloudCentroidNode : public rclcpp::Node
{
public:
    PointCloudCentroidNode() : Node("point_cloud_centroid"), is_ptcld_(false), display_(true)
    {
        // Publishers
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/object_centroid", 10);

        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/er_kachaka/back_camera/image_raw", 10, 
            std::bind(&PointCloudCentroidNode::callback_rgb_rect, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/er_kachaka/tof_camera/image_raw", 10, 
            std::bind(&PointCloudCentroidNode::callback_depth_rect, this, std::placeholders::_1));
        
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/er_kachaka/tof_camera/image_raw/compressedDepth", 10, 
            std::bind(&PointCloudCentroidNode::callback_compressed_image, this, std::placeholders::_1));
        
        // Processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&PointCloudCentroidNode::point_cloud_processing, this));

        RCLCPP_INFO(this->get_logger(), "Starting point_cloud_centroid application in cpp...");
    }

private:

    bool is_ptcld_;
    bool display_;
    cv::Mat rgb_, depth_, depth_mat_, point_cloud_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr centroid_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr point_cloud_sub_;

    rclcpp::TimerBase::SharedPtr processing_timer_;

    void callback_rgb_rect(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        try 
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            rgb_ = cv_ptr->image.clone();
            cv::cvtColor(rgb_, rgb_, cv::COLOR_RGB2BGR);
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGB image: %s", e.what());
        }
    }

    void callback_depth_rect(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        try 
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
            depth_ = cv_ptr->image.clone();

            depth_mat_ = cv::Mat(depth_.size(), CV_32F);
            depth_.convertTo(depth_mat_, CV_32F);
            cv::normalize(depth_mat_, depth_, 0, 1, cv::NORM_MINMAX);
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing Depth image: %s", e.what());
        }
    }

    void callback_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg) 
    {
        try
        {
            // Decode the compressed image data into a cv::Mat
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

            if (!image.empty())
            {
                // Process the image as needed
                RCLCPP_INFO(this->get_logger(), "Compressed image received with size [%d, %d]", image.rows, image.cols);

                // Example: Convert to grayscale
                cv::Mat gray_image;
                cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

                // You can add further processing here
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to decode compressed image.");
            }
        }
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing compressed image: %s", e.what());
        }
    }

    void point_cloud_processing() 
    {
        if (is_ptcld_) 
        {
            // Access a point in the point_cloud (e.g., midpoint of the matrix)
            int rows = point_cloud_.rows;
            int cols = point_cloud_.cols;
            int row_id = rows / 2;
            int col_id = cols / 2;

            // Ensure the indices are within valid bounds
            if (row_id >= 0 && row_id < rows && col_id >= 0 && col_id < cols) 
            {
                // Extract the point at the specified location (row_id, col_id)
                cv::Vec4f point = point_cloud_.at<cv::Vec4f>(row_id, col_id);

                // Assign the extracted point's coordinates to centroid_
                geometry_msgs::msg::Pose centroid;

                centroid.position.x = static_cast<float>(point[0]); // x
                centroid.position.y = static_cast<float>(point[1]); // y
                centroid.position.z = static_cast<float>(point[2]); // z

                // Set a default orientation
                centroid.orientation.x = 0.0;
                centroid.orientation.y = 0.0;
                centroid.orientation.z = 0.0;
                centroid.orientation.w = 1.0;

                RCLCPP_INFO(this->get_logger(), "Central point: x=%.3f, y=%.3f, z=%.3f", point[0], point[1], point[2]);

                // Publish the centroid pose
                centroid_publisher_->publish(centroid);
            } 
            else 
            {
                RCLCPP_WARN(this->get_logger(), "PointCloud index out of bounds");
            }

            if (display_) 
            {
                cv::imshow("RGB Image", rgb_);
                cv::imshow("Depth Image", depth_);
                cv::waitKey(1);
            }
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "Empty point cloud matrix.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudCentroidNode>();
    rclcpp::spin(node);

    cv::destroyAllWindows();
    rclcpp::shutdown();
    
    return 0;
}
