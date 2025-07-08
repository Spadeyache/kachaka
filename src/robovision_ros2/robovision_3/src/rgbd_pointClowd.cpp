#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class RGBDPointCloudNode : public rclcpp::Node
{
public:
    RGBDPointCloudNode() : Node("rgbd_point_cloud_node")
    {
        // Subscribers
        rgb_sub_.subscribe(this, "/er_kachaka/front_camera/image_raw");
        depth_sub_.subscribe(this, "/er_kachaka/tof_camera/image_raw");

        // Synchronizer
        sync_.reset(new Sync(MySyncPolicy(10), rgb_sub_, depth_sub_));
        sync_->registerCallback(std::bind(&RGBDPointCloudNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/kn_point_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "Starting rgbd_point_cloud_node application in cpp...");
    }

private:
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    void callback(const sensor_msgs::msg::Image::SharedPtr rgb_msg, const sensor_msgs::msg::Image::SharedPtr depth_msg)
    {
        try
        {
            cv_bridge::CvImagePtr rgb_cv_ptr = cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
            cv_bridge::CvImagePtr depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, "32FC1");

            cv::Mat rgb_image = rgb_cv_ptr->image;
            cv::Mat depth_image = depth_cv_ptr->image;

            if (rgb_image.empty() || depth_image.empty())
            {
                RCLCPP_WARN(this->get_logger(), "RGB or Depth image is empty.");
                return;
            }

            // Compute point cloud
            compute_point_cloud(rgb_image, depth_image);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing images: %s", e.what());
        }
    }

    void compute_point_cloud(const cv::Mat &rgb_image, const cv::Mat &depth_image)
    {
        // Create a PointCloud2 message
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        point_cloud_msg.header.frame_id = "camera_frame";
        point_cloud_msg.height = depth_image.rows;
        point_cloud_msg.width = depth_image.cols;
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

        for (int v = 0; v < depth_image.rows; ++v)
        {
            for (int u = 0; u < depth_image.cols; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
            {
                float depth = depth_image.at<float>(v, u);
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
                    cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(v, u);
                    *iter_r = rgb[2]; // Red channel
                    *iter_g = rgb[1]; // Green channel
                    *iter_b = rgb[0]; // Blue channel
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
