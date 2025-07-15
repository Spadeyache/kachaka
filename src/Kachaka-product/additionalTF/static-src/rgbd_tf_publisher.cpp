#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
  StaticFramePublisher()
  : Node("rgbd_tf_publisher")
  {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped static_transform;

    static_transform.header.stamp = this->get_clock()->now();
    static_transform.header.frame_id = "base_link";
    static_transform.child_frame_id = "camera_link";

    static_transform.transform.translation.x = 0.1;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.2;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);  // roll, pitch, yaw
    static_transform.transform.rotation.x = quat.x();
    static_transform.transform.rotation.y = quat.y();
    static_transform.transform.rotation.z = quat.z();
    static_transform.transform.rotation.w = quat.w();

    static_broadcaster_->sendTransform(static_transform);
    RCLCPP_INFO(this->get_logger(), "Static transform from base_link to camera_link published.");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}
