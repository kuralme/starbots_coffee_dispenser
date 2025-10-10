#include "bt_nodes/validate_detection_node.hpp"

ValidateDetection::ValidateDetection(const std::string &name,
                                     const BT::NodeConfig &config,
                                     PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList ValidateDetection::providedPorts() { return {}; }

BT::NodeStatus ValidateDetection::tick() {
  auto blackboard_ = config().blackboard;
  auto request =
      blackboard_->get<std::shared_ptr<DeliverCup::Request>>("request");

  robot_->update_goals_ = true; // Enable to populate detection
  int no_detection_count = 0;
  const int max_attempts = 5;

  robot_->publishStatus("Checking detection...", "INFO");

  while (robot_->goal_poses_.empty()) {
    RCLCPP_WARN(LOGGER, "Detection data not received yet (attempt %d/%d)",
                no_detection_count + 1, max_attempts);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    no_detection_count++;

    if (no_detection_count >= max_attempts) {
      robot_->publishStatus("Detection timeout after attempts", "ERROR");
      robot_->update_goals_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  robot_->update_goals_ = false;
  auto goal_position = robot_->goal_poses_[request->goal_cup_holder];
  if (goal_position.x > -0.35) {
    robot_->publishStatus("Cuphole not reachable - Too close!", "ERROR");
    return BT::NodeStatus::FAILURE;

  } else if (goal_position.x < -0.49 || goal_position.y > 0.4) {
    robot_->publishStatus("Cuphole not reachable - Too far!", "ERROR");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}