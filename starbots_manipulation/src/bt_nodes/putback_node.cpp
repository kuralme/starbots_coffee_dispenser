#include "bt_nodes/putback_node.hpp"

PutBack::PutBack(const std::string &name, const BT::NodeConfig &config,
                 PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PutBack::providedPorts() { return {}; }

BT::NodeStatus PutBack::tick() {

  auto blackboard_ = config().blackboard;
  blackboard_->set("place_failed", true);

  RCLCPP_INFO(LOGGER, "Putting the cup back to original position");
  geometry_msgs::msg::Point prepick_pos = robot_->cup_position_;
  prepick_pos.z += 0.3;

  robot_->createOrientationConstraint();
  auto future_result = std::async(std::launch::async, [&]() {
    return robot_->executeKinematicsPlan(prepick_pos.x, prepick_pos.y,
                                         prepick_pos.z);
  });
  // Timeout defined to avoid hanging indefinitely
  if (future_result.wait_for(std::chrono::seconds(30)) !=
          std::future_status::ready ||
      !future_result.get()) {

    // Try again from Intermediate pose
    robot_->executeKinematicsPlan(-0.200, 0.150, 0.400);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    robot_->executeKinematicsPlan(prepick_pos.x, prepick_pos.y, prepick_pos.z);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeCartesianPlan(+0.000, +0.000, -0.075);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeGripperPlan("gripper_open");
  robot_->detachGripperObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeCartesianPlan(+0.000, +0.000, +0.075);
  robot_->clearOrientationConstraints();
  return BT::NodeStatus::SUCCESS;
}