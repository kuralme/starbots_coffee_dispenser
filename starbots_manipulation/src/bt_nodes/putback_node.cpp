#include "bt_nodes/putback_node.hpp"

PutBack::PutBack(const std::string &name, const BT::NodeConfig &config,
                 PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PutBack::providedPorts() {
  return {BT::InputPort<bool>("place_failed")};
}

BT::NodeStatus PutBack::tick() {
  geometry_msgs::msg::Point prepick_pos = robot_->cup_position_;
  prepick_pos.z += 0.3;

  RCLCPP_INFO(LOGGER, "Putting the cup back to original position");

  robot_->createOrientationConstraint();
  robot_->executeKinematicsPlan(prepick_pos.x, prepick_pos.y, prepick_pos.z);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeCartesianPlan(+0.000, +0.000, -0.070);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeGripperPlan("gripper_open");
  robot_->detachGripperObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeCartesianPlan(+0.000, +0.000, +0.070);
  robot_->clearOrientationConstraints();

  auto blackboard_ = config().blackboard;
  blackboard_->set("place_failed", true);
  return BT::NodeStatus::FAILURE;
}