#include "bt_nodes/place_node.hpp"

Place::Place(const std::string &name, const BT::NodeConfig &config,
             PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Place::providedPorts() { return {}; }

BT::NodeStatus Place::tick() {

  auto blackboard_ = config().blackboard;
  auto request =
      blackboard_->get<std::shared_ptr<DeliverCup::Request>>("request");
  auto goal_position = robot_->goal_poses_[request->goal_cup_holder];
  goal_position.z += .3;

  RCLCPP_INFO(LOGGER, "Approaching to place...");

  robot_->createOrientationConstraint();
  robot_->move_group_robot_->setPlanningTime(10.0);
  if (!robot_->executeKinematicsPlan(goal_position.x, goal_position.y,
                                     goal_position.z, 5)) {
    RCLCPP_ERROR(LOGGER, "Place kinematics plan failed!");
    robot_->move_group_robot_->stop();
    robot_->clearOrientationConstraints();
    robot_->move_group_robot_->setPlanningTime(20.0); // Default for next
    return BT::NodeStatus::FAILURE;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  robot_->executeGripperPlan("gripper_open");
  robot_->detachCollisionObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Retreating...");
  if (!robot_->executeKinematicsPlan(goal_position.x, goal_position.y,
                                     goal_position.z + .2, 10)) {
    RCLCPP_ERROR(LOGGER, "Retreat kinematics plan failed!");
    robot_->move_group_robot_->stop();
    robot_->clearOrientationConstraints();
    robot_->move_group_robot_->setPlanningTime(20.0); // Default for next
    return BT::NodeStatus::FAILURE;
  }

  robot_->clearOrientationConstraints();
  robot_->move_group_robot_->setPlanningTime(20.0); // Default
  return BT::NodeStatus::SUCCESS;
}