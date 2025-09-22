#include "bt_nodes/pick_node.hpp"

Pick::Pick(const std::string &name, const BT::NodeConfig &config,
           PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Pick::providedPorts() { return {}; }

BT::NodeStatus Pick::tick() {

  RCLCPP_INFO(LOGGER, "Approaching to pick...");
  robot_->executeGripperPlan("gripper_open");
  robot_->executeCartesianPlan(+0.000, +0.000, -0.075);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeGripperPlan("gripper_grasp");
  robot_->attachGripperObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeCartesianPlan(+0.000, +0.000, +0.075);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  return BT::NodeStatus::SUCCESS;
}