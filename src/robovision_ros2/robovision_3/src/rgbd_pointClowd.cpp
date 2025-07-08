#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class RGBDPointCloudNode : public rclcpp::Node
{
public:
    RGBDPointCloudNode() : Node("rgbd_point_cloud_node")
    {
        // Subscribers
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10, 
            std::bind(&RGBDPointCloudNode::callback_rgb, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, 
            std::bind(&RGBDPointCloudNode::callback_depth, this, std::placeholders::_1));
        
        // Publisher
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/point_cloud", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    cv::Mat rgb_image_;
    cv::Mat depth_image_;

    void callback_rgb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            rgb_image_ = cv_ptr->image.clone();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGB image: %s", e.what());
        }
    }

    void callback_depth(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
            depth_image_ = cv_ptr->image.clone();
            compute_point_cloud();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing Depth image: %s", e.what());
        }
    }

    void compute_point_cloud()
    {
        if (rgb_image_.empty() || depth_image_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "RGB or Depth image is empty.");
            return;
        }

        // Create a PointCloud2 message
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        point_cloud_msg.header.frame_id = "camera_frame";
        point_cloud_msg.height = depth_image_.rows;
        point_cloud_msg.width = depth_image_.cols;
        point_cloud_msg.is_dense = false;
        point_cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_msg, "b");

        for (int v = 0; v < depth_image_.rows; ++v)
        {
            for (int u = 0; u < depth_image_.cols; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
            {
                float depth = depth_image_.at<float>(v, u);
                if (std::isnan(depth) || depth <= 0.0)
                {
                    *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
                }
                else
                {
                    // Compute 3D coordinates (x, y, z) from depth
                    *iter_x = (u - cx) * depth / fx;
                    *iter_y = (v - cy) * depth / fy;
                    *iter_z = depth;

                    // Assign RGB values
                    cv::Vec3b rgb = rgb_image_.at<cv::Vec3b>(v, u);
                    *iter_r = rgb[2];
                    *iter_g = rgb[1];
                    *iter_b = rgb[0];
                }
            }
        }

        point_cloud_pub_->publish(point_cloud_msg);
    }

    // Camera intrinsic parameters (example values, replace with your camera's parameters)
    const float fx = 525.0; // Focal length x
    const float fy = 525.0; // Focal length y
    const float cx = 319.5; // Optical center x
    const float cy = 239.5; // Optical center y
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBDPointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
