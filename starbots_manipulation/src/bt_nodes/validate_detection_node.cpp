#include "bt_nodes/validate_detection_node.hpp"

ValidateDetection::ValidateDetection(const std::string &name,
                                     const BT::NodeConfig &config,
                                     PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList ValidateDetection::providedPorts() { return {}; }

BT::NodeStatus ValidateDetection::tick() {
  robot_->update_goals_ = true; // Enable to populate detection
  int no_detection_count = 0;
  const int max_attempts = 5;

  while (robot_->goal_poses_.empty()) {
    RCLCPP_WARN(LOGGER, "Detection data not received yet (attempt %d/%d)",
                no_detection_count + 1, max_attempts);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    no_detection_count++;

    if (no_detection_count >= max_attempts) {
      RCLCPP_ERROR(LOGGER, "Detection timeout after %d attempts", max_attempts);
      robot_->update_goals_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }
  robot_->update_goals_ = false;
  return BT::NodeStatus::SUCCESS;
}