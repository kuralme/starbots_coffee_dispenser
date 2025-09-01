#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudFilter : public rclcpp::Node {
public:
  PointCloudFilter() : Node("pointcloud_filter_node") {
    tablecam_filtered_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera_depth_sensor/points_filtered", 10);
    baristacam_filtered_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/wrist_rgbd_depth_sensor/points_filtered", 10);
    tablecam_raw_sub_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_depth_sensor/points", 10,
            std::bind(&PointCloudFilter::tableCloudCallback, this,
                      std::placeholders::_1));
    baristacam_raw_sub_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wrist_rgbd_depth_sensor/points", 10,
            std::bind(&PointCloudFilter::baristaCloudCallback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Point Cloud filtering node initialized");
  }

private:
  void tableCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    sensor_msgs::msg::PointCloud2 filtered_cloud;
    filtered_cloud.header = msg->header;
    filtered_cloud.is_dense = msg->is_dense;
    filtered_cloud.is_bigendian = msg->is_bigendian;
    filtered_cloud.fields = msg->fields;
    filtered_cloud.point_step = msg->point_step;
    filtered_cloud.row_step = 0;
    filtered_cloud.data.clear();
    filtered_cloud.data.reserve(msg->data.size());
    size_t valid_points = 0;

    // Iterate through the point cloud and apply the filter
    for (size_t i = 0; i < msg->width * msg->height;
         ++i, ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isinf(x) ||
          std::isinf(y) || std::isinf(z)) {
        continue; // Skip invalid points
      }

      if (y >= -0.3) {
        filtered_cloud.data.insert(
            filtered_cloud.data.end(), msg->data.begin() + i * msg->point_step,
            msg->data.begin() + (i + 1) * msg->point_step);
        ++valid_points;
      }
    }

    // Update cloud properties
    if (valid_points > 0) {
      filtered_cloud.width = valid_points;
      filtered_cloud.height = 1;
      filtered_cloud.row_step =
          filtered_cloud.width * filtered_cloud.point_step;

      tablecam_filtered_pub_->publish(filtered_cloud);
    } else {
      RCLCPP_WARN(this->get_logger(), "No points left after filtering.");
    }
  }
  void
  baristaCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    sensor_msgs::msg::PointCloud2 filtered_cloud;
    filtered_cloud.header = msg->header;
    filtered_cloud.is_dense = msg->is_dense;
    filtered_cloud.is_bigendian = msg->is_bigendian;
    filtered_cloud.fields = msg->fields;
    filtered_cloud.point_step = msg->point_step;
    filtered_cloud.row_step = 0;
    filtered_cloud.data.clear();
    filtered_cloud.data.reserve(msg->data.size());
    size_t valid_points = 0;

    // Iterate through the point cloud and apply the filter
    for (size_t i = 0; i < msg->width * msg->height;
         ++i, ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isinf(x) ||
          std::isinf(y) || std::isinf(z)) {
        continue; // Skip invalid points
      }

      if (x >= -0.35 && y >= -0.15 && y <= 0.3 && z <= 0.9) {
        filtered_cloud.data.insert(
            filtered_cloud.data.end(), msg->data.begin() + i * msg->point_step,
            msg->data.begin() + (i + 1) * msg->point_step);
        ++valid_points;
      }
    }

    // Update cloud properties
    if (valid_points > 0) {
      filtered_cloud.width = valid_points;
      filtered_cloud.height = 1;
      filtered_cloud.row_step =
          filtered_cloud.width * filtered_cloud.point_step;

      baristacam_filtered_pub_->publish(filtered_cloud);
    } else {
      RCLCPP_WARN(this->get_logger(), "No points left after filtering.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      tablecam_raw_sub_,
      baristacam_raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tablecam_filtered_pub_,
      baristacam_filtered_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
