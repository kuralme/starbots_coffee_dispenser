#include "bt_nodes/place_node.hpp"

Place::Place(const std::string &name, const BT::NodeConfig &config,
             PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Place::providedPorts() { return {}; }

BT::NodeStatus Place::tick() {

  RCLCPP_INFO(LOGGER, "Approaching to place...");
  geometry_msgs::msg::Pose robot_pose =
      robot_->move_group_robot_->getCurrentPose().pose;

  robot_->createOrientationConstraint();
  // robot_->clearOctomap();
  if (!robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z - 0.230)) {
    robot_->move_group_robot_->clearPoseTargets();
    robot_->executeCartesianPlan(+0.000, +0.000, -0.230);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  robot_->executeGripperPlan("gripper_open");
  robot_->detachGripperObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  RCLCPP_INFO(LOGGER, "Retreating...");
  if (!robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z)) {
    robot_->move_group_robot_->clearPoseTargets();
    robot_->executeCartesianPlan(+0.000, +0.000, +0.300);
  }

  robot_->clearOrientationConstraints();
  return BT::NodeStatus::SUCCESS;
}