#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class RGBDPointCloudPublisher : public rclcpp::Node {
public:
  RGBDPointCloudPublisher()
  : Node("rgbd_pointcloud_publisher")
  {
    rgb_sub_.subscribe(this, "/er_kachaka/image_raw");
    depth_sub_.subscribe(this, "/er_kachaka/tof_camera/image_raw");
    info_sub_.subscribe(this, "/er_kachaka/tof_camera/image_raw/camera_info");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_);
    sync_->registerCallback(std::bind(
      &RGBDPointCloudPublisher::callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    RCLCPP_INFO(this->get_logger(), "RGBD PointCloud2 Publisher initialized.");
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo
  > SyncPolicy;

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
  {
    cv::Mat rgb_img, depth_img;

    try {
      rgb_img = cv_bridge::toCvCopy(rgb_msg, "rgb8")->image;
      depth_img = cv_bridge::toCvCopy(depth_msg, "passthrough")->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    float fx = info_msg->k[0];
    float fy = info_msg->k[4];
    float cx = info_msg->k[2];
    float cy = info_msg->k[5];

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = depth_msg->header;
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;

    std::vector<std::tuple<float, float, float, uint32_t>> points;

    for (int v = 0; v < depth_img.rows; ++v) {
      for (int u = 0; u < depth_img.cols; ++u) {
        float z = depth_img.at<float>(v, u);
        if (z <= 0 || std::isnan(z)) continue;

        float x = (u - cx) * z / fx;
        float y = (v - cy) * z / fy;

        cv::Vec3b rgb = rgb_img.at<cv::Vec3b>(v, u);
        uint32_t rgb_packed = ((uint32_t)rgb[2] << 16 | (uint32_t)rgb[1] << 8 | (uint32_t)rgb[0]);

        points.emplace_back(x, y, z, rgb_packed);
      }
    }

    cloud_msg.width = points.size();
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz", "rgb");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_msg, "rgb");

    for (const auto& p : points) {
      *iter_x = std::get<0>(p);
      *iter_y = std::get<1>(p);
      *iter_z = std::get<2>(p);
      uint32_t rgb = std::get<3>(p);
      iter_rgb[0] = (rgb >> 16) & 0xFF;
      iter_rgb[1] = (rgb >> 8) & 0xFF;
      iter_rgb[2] = rgb & 0xFF;

      ++iter_x; ++iter_y; ++iter_z;
      ++iter_rgb;
    }

    pc_pub_->publish(cloud_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBDPointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
