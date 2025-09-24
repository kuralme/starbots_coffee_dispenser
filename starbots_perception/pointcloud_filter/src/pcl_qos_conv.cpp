#include "pcl/point_cloud.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PclConv : public rclcpp::Node {
public:
  PclConv() : Node("pcl_qos_conv_node") {
    rclcpp::QoS pub_qos(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    rclcpp::QoS sub_qos(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    baristacam_conv_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/D415/barista_points", pub_qos);
    baristacam_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/D415/barista_points_beff", sub_qos,
        std::bind(&PclConv::baristaPclCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Point Cloud QoS converter node initialized");
  }

private:
  void baristaPclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Simply forwarding the message with different QoS profile
    baristacam_conv_pub_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      baristacam_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      baristacam_conv_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclConv>());
  rclcpp::shutdown();
  return 0;
}
