#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class DepthViewer : public rclcpp::Node {
public:
  DepthViewer() : Node("depth_viewer") {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/er_kachaka/tof_camera/image_raw", qos,
      std::bind(&DepthViewer::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv::Mat depth;
      auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

      if (msg->encoding == "16UC1") {
        cv_ptr->image.convertTo(depth, CV_32FC1, 0.001); // mm → m
      } else if (msg->encoding == "32FC1") {
        depth = cv_ptr->image;
      } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
        return;
      }

      float z = depth.at<float>(depth.rows / 2, depth.cols / 2);
      RCLCPP_INFO(this->get_logger(), "Center depth value: %.3f m", z);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthViewer>());
  rclcpp::shutdown();
  return 0;
}
