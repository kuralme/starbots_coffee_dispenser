#include "bt_nodes/return_node.hpp"

Return::Return(const std::string &name, const BT::NodeConfig &config,
               PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Return::providedPorts() { return {}; }

BT::NodeStatus Return::tick() {
  robot_->goal_poses_received_ = false;
  robot_->gotoPredefined("quick_pick");

  bool place_failed = false;
  auto blackboard_ = config().blackboard;
  blackboard_->get("place_failed", place_failed);
  return place_failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}