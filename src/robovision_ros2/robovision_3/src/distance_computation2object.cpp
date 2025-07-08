#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>

class DistanceComputationNode : public rclcpp::Node {
public:
    DistanceComputationNode() : Node("distance_computation") {
        RCLCPP_INFO(this->get_logger(), "Distance Computation Node initialized.");

        center_of_mass_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/kn_center_of_mass", 10,
            std::bind(&DistanceComputationNode::centerOfMassCallback, this, std::placeholders::_1));

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/kn_pointcloud2", 10,
            std::bind(&DistanceComputationNode::pointCloudCallback, this, std::placeholders::_1));

        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/object_distance", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_of_mass_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;

    geometry_msgs::msg::Point::SharedPtr latest_center_of_mass_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;

    void centerOfMassCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_center_of_mass_ = msg;
        computeDistance();
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_pointcloud_ = msg;
        computeDistance();
    }

    void computeDistance() {
        if (!latest_center_of_mass_ || !latest_pointcloud_) {
            return;
        }

        // Assuming the center of mass coordinates are in the same frame as the point cloud
        int u = static_cast<int>(latest_center_of_mass_->x);
        int v = static_cast<int>(latest_center_of_mass_->y);

        // Ensure the indices are within the point cloud dimensions
        if (u < 0 || v < 0 || u >= latest_pointcloud_->width || v >= latest_pointcloud_->height) {
            RCLCPP_WARN(this->get_logger(), "Center of mass is out of point cloud bounds.");
            return;
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_pointcloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_pointcloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_pointcloud_, "z");

        // Calculate the index in the point cloud data
        int index = v * latest_pointcloud_->width + u;
        std::advance(iter_x, index);
        std::advance(iter_y, index);
        std::advance(iter_z, index);

        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        // Compute the distance
        float distance = std::sqrt(x * x + y * y + z * z);

        // Publish the distance
        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = distance;
        distance_pub_->publish(distance_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceComputationNode>());
    rclcpp::shutdown();
    return 0;
} 