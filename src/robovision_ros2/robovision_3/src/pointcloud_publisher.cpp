#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RGBDToPointCloud : public rclcpp::Node {
public:
  RGBDToPointCloud() : Node("rgbd_pointcloud_publisher") {
    RCLCPP_INFO(this->get_logger(), "PointCloud Publisher initialized.");

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/er_kachaka/front_camera/image_raw", qos,
      std::bind(&RGBDToPointCloud::rgb_callback, this, std::placeholders::_1));

    rgb_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/er_kachaka/front_camera/camera_info", qos,
      std::bind(&RGBDToPointCloud::rgb_info_callback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/er_kachaka/tof_camera/image_raw", qos,
      std::bind(&RGBDToPointCloud::depth_callback, this, std::placeholders::_1));

    depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/er_kachaka/tof_camera/camera_info", qos,
      std::bind(&RGBDToPointCloud::depth_info_callback, this, std::placeholders::_1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/kn_pointcloud2", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  cv::Mat latest_rgb_;
  rclcpp::Time last_rgb_time_;

  bool has_rgb_info_ = false;
  bool has_depth_info_ = false;
  float fx_ = 0.0f, fy_ = 0.0f, cx_ = 0.0f, cy_ = 0.0f;

  void rgb_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!has_rgb_info_) {
      fx_ = msg->k[0];
      fy_ = msg->k[4];
      cx_ = msg->k[2];
      cy_ = msg->k[5];
      has_rgb_info_ = true;
      RCLCPP_INFO(this->get_logger(), "RGB CameraInfo received.");
    }
  }

  void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!has_depth_info_) {
      fx_ = msg->k[0];
      fy_ = msg->k[4];
      cx_ = msg->k[2];
      cy_ = msg->k[5];
      has_depth_info_ = true;
      RCLCPP_INFO(this->get_logger(), "Depth CameraInfo received.");
    }
  }

  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      latest_rgb_ = cv_ptr->image;
      last_rgb_time_ = msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), "RGB image received.");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in RGB callback: %s", e.what());
    }
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!has_depth_info_ || latest_rgb_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Waiting for camera info or RGB.");
      return;
    }

    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      cv::Mat depth;
      if (msg->encoding == "16UC1") {
        cv_ptr->image.convertTo(depth, CV_32FC1, 0.001); // mm → meters
      } else if (msg->encoding == "32FC1") {
        depth = cv_ptr->image;
      } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported depth encoding: %s", msg->encoding.c_str());
        return;
      }

      // Resize RGB if needed
      cv::Mat rgb_resized;
      if (latest_rgb_.rows != depth.rows || latest_rgb_.cols != depth.cols) {
        RCLCPP_WARN(this->get_logger(), "Resizing RGB to match Depth");
        cv::resize(latest_rgb_, rgb_resized, depth.size(), 0, 0, cv::INTER_LINEAR);
      } else {
        rgb_resized = latest_rgb_;
      }

      cv::cvtColor(rgb_resized, rgb_resized, cv::COLOR_BGR2RGB);

      // Create PointCloud2 message
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_msg.header.stamp = this->get_clock()->now();
      cloud_msg.header.frame_id = "tof_camera";
      cloud_msg.height = 1;
      cloud_msg.width = depth.rows * depth.cols;
      cloud_msg.is_dense = false;
      cloud_msg.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      modifier.resize(cloud_msg.width);

      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg, "rgb");

      for (int v = 0; v < depth.rows; ++v) {
        for (int u = 0; u < depth.cols; ++u) {
          float z = depth.at<float>(v, u);
          if (z == 0.0f || std::isnan(z)) {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
            *iter_rgb = 0.0f;
          } else {
            float x = (u - cx_) * z / fx_;
            float y = (v - cy_) * z / fy_;
            *iter_x = x;
            *iter_y = y;
            *iter_z = z;

            uint8_t r = rgb_resized.at<cv::Vec3b>(v, u)[0];
            uint8_t g = rgb_resized.at<cv::Vec3b>(v, u)[1];
            uint8_t b = rgb_resized.at<cv::Vec3b>(v, u)[2];
            uint32_t rgb = (r << 16) | (g << 8) | b;
            *iter_rgb = *reinterpret_cast<float*>(&rgb);
          }

          ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
        }
      }

      pc_pub_->publish(cloud_msg);
      RCLCPP_INFO(this->get_logger(), "Published point cloud with %d points", cloud_msg.width);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in Depth callback: %s", e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBDToPointCloud>());
  rclcpp::shutdown();
  return 0;
}



// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/point_cloud2_iterator.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.hpp>
// #include <opencv2/opencv.hpp>

// class RGBDToPointCloud : public rclcpp::Node {
// public:
//   RGBDToPointCloud() : Node("rgbd_pointcloud_publisher") {
//     RCLCPP_INFO(this->get_logger(), "PointCloud Publisher initialized.");

//     auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
//     qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

//     rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//       "/er_kachaka/front_camera/image_raw", qos,
//       std::bind(&RGBDToPointCloud::rgb_callback, this, std::placeholders::_1));

//     depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//       "/er_kachaka/tof_camera/image_raw", qos,
//       std::bind(&RGBDToPointCloud::depth_callback, this, std::placeholders::_1));

//     pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
//   }

// private:
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

//   cv::Mat latest_rgb_;
//   rclcpp::Time last_rgb_time_;

//   void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     try {
//       auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
//       latest_rgb_ = cv_ptr->image;
//       last_rgb_time_ = msg->header.stamp;
//       RCLCPP_INFO(this->get_logger(), "RGB image received.");
//     } catch (cv_bridge::Exception &e) {
//       RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in RGB callback: %s", e.what());
//     }
//   }

//   void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     if (latest_rgb_.empty()) {
//       RCLCPP_WARN(this->get_logger(), "No RGB image received yet.");
//       return;
//     }

//     try {
//       auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
//       cv::Mat depth;

//       if (msg->encoding == "16UC1") {
//         cv_ptr->image.convertTo(depth, CV_32FC1, 0.001);  // mm → meters
//       } else if (msg->encoding == "32FC1") {
//         depth = cv_ptr->image;
//       } else {
//         RCLCPP_WARN(this->get_logger(), "Unsupported depth encoding: %s", msg->encoding.c_str());
//         return;
//       }

//       // Resize RGB if necessary
//       cv::Mat rgb_resized;
//       if (latest_rgb_.rows != depth.rows || latest_rgb_.cols != depth.cols) {
//         RCLCPP_WARN(this->get_logger(), "Resizing RGB to match Depth");
//         cv::resize(latest_rgb_, rgb_resized, depth.size(), 0, 0, cv::INTER_LINEAR);
//       } else {
//         rgb_resized = latest_rgb_;
//       }

//       // Convert BGR to RGB
//       cv::cvtColor(rgb_resized, rgb_resized, cv::COLOR_BGR2RGB);

//       sensor_msgs::msg::PointCloud2 cloud_msg;
//       cloud_msg.header.stamp = this->get_clock()->now();
//       cloud_msg.header.frame_id = "tof_camera";
//       cloud_msg.height = 1;
//       cloud_msg.width = depth.rows * depth.cols;
//       cloud_msg.is_bigendian = false;
//       cloud_msg.is_dense = false;

//       sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
//       modifier.setPointCloud2Fields(
//         4,
//         "x", 1, sensor_msgs::msg::PointField::FLOAT32,
//         "y", 1, sensor_msgs::msg::PointField::FLOAT32,
//         "z", 1, sensor_msgs::msg::PointField::FLOAT32,
//         "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
//       modifier.resize(cloud_msg.width);

//       sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
//       sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
//       sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
//       sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg, "rgb");

//       for (int v = 0; v < depth.rows; ++v) {
//         for (int u = 0; u < depth.cols; ++u) {
//           float z = depth.at<float>(v, u);
//           if (z == 0.0f || std::isnan(z)) {
//             *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
//             *iter_rgb = 0.0f;
//           } else {
//             float x = (u - depth.cols / 2.0f) * z / 500.0f;  // fx = ~500 (you can adjust)
//             float y = (v - depth.rows / 2.0f) * z / 500.0f;

//             *iter_x = x;
//             *iter_y = y;
//             *iter_z = z;

//             uint8_t r = rgb_resized.at<cv::Vec3b>(v, u)[0];
//             uint8_t g = rgb_resized.at<cv::Vec3b>(v, u)[1];
//             uint8_t b = rgb_resized.at<cv::Vec3b>(v, u)[2];

//             uint32_t rgb = (r << 16) | (g << 8) | b;
//             *iter_rgb = *reinterpret_cast<float*>(&rgb);
//           }

//           ++iter_x;
//           ++iter_y;
//           ++iter_z;
//           ++iter_rgb;
//         }
//       }

//       pc_pub_->publish(cloud_msg);
//       RCLCPP_INFO(this->get_logger(), "Published point cloud with %d points", cloud_msg.width);

//     } catch (cv_bridge::Exception &e) {
//       RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in Depth callback: %s", e.what());
//     }
//   }
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<RGBDToPointCloud>());
//   rclcpp::shutdown();
//   return 0;
// }



