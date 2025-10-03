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
  robot_->move_group_robot_->setPlanningTime(5.0);

  if (!robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z - 0.230) &&
      !robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z - 0.215) &&
      !robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z - 0.200)) {
    RCLCPP_ERROR(LOGGER, "Place kinematics plan failed!");
    robot_->move_group_robot_->stop();
    robot_->clearOrientationConstraints();
    robot_->move_group_robot_->setPlanningTime(20.0); // For future runs
    return BT::NodeStatus::FAILURE;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  robot_->executeGripperPlan("gripper_open");
  robot_->detachCollisionObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Retreating...");
  if (!robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z) &&
      !robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z - 0.02) &&
      !robot_->executeKinematicsPlan(robot_pose.position.x,
                                     robot_pose.position.y,
                                     robot_pose.position.z + 0.02)) {
    RCLCPP_ERROR(LOGGER, "Retreat kinematics plan failed!");
    robot_->move_group_robot_->stop();
    robot_->clearOrientationConstraints();
    robot_->move_group_robot_->setPlanningTime(20.0); // For future runs
    return BT::NodeStatus::FAILURE;
  }

  robot_->clearOrientationConstraints();
  robot_->move_group_robot_->setPlanningTime(20.0); // Default
  return BT::NodeStatus::SUCCESS;
}