#include "bt_nodes/prepick_node.hpp"

PrePick::PrePick(const std::string &name, const BT::NodeConfig &config,
                 PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PrePick::providedPorts() { return {}; }

BT::NodeStatus PrePick::tick() {
  geometry_msgs::msg::Point prepick_pos = robot_->cup_position_;
  prepick_pos.z += 0.3;

  RCLCPP_INFO(LOGGER, "Going to Pre-pick Pose: [%.3f, %.3f, %.3f]",
              prepick_pos.x, prepick_pos.y, prepick_pos.z);

  // Timeout after 20 seconds to avoid hanging indefinitely
  auto future_result = std::async(std::launch::async, [&]() {
    return robot_->executeKinematicsPlan(prepick_pos.x, prepick_pos.y,
                                         prepick_pos.z);
  });
  if (future_result.wait_for(std::chrono::seconds(20)) !=
          std::future_status::ready ||
      !future_result.get()) {
    RCLCPP_ERROR(LOGGER, "Prepick: Kinematics plan failed or timed out.");
    robot_->move_group_robot_->stop();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}