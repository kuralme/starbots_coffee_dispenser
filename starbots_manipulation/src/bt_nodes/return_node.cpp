#include "bt_nodes/return_node.hpp"

Return::Return(const std::string &name, const BT::NodeConfig &config,
               PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Return::providedPorts() { return {}; }

BT::NodeStatus Return::tick() {
  robot_->goal_poses_.clear();
  robot_->gotoPredefined("quick_pick");

  auto blackboard_ = config().blackboard;
  auto place_failed = blackboard_->get<bool>("place_failed");
  return place_failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}