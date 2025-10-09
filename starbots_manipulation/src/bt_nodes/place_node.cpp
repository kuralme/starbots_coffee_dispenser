#include "bt_nodes/place_node.hpp"

Place::Place(const std::string &name, const BT::NodeConfig &config,
             PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Place::providedPorts() { return {}; }

BT::NodeStatus Place::tick() {

  auto blackboard_ = config().blackboard;
  auto request =
      blackboard_->get<std::shared_ptr<DeliverCup::Request>>("request");
  auto prepick_position = robot_->goal_poses_[request->goal_cup_holder];
  prepick_position.z += .3;
  prepick_position.x = std::clamp(prepick_position.x, -1.,
                                  -0.35); // Added due to robot limitations

  RCLCPP_INFO(LOGGER, "Approaching to place...");

  robot_->createOrientationConstraint();
  robot_->move_group_robot_->setPlanningTime(10.0);
  if (!robot_->executeKinematicsPlan(prepick_position.x, prepick_position.y,
                                     prepick_position.z, 5)) {
    RCLCPP_ERROR(LOGGER, "Place kinematics plan failed!");
    robot_->move_group_robot_->stop();
    robot_->clearOrientationConstraints();
    robot_->move_group_robot_->setPlanningTime(20.0); // Default for next
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return BT::NodeStatus::FAILURE;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  robot_->executeGripperPlan("gripper_open");
  robot_->detachCollisionObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Retreating...");
  if (!robot_->executeKinematicsPlan(prepick_position.x, prepick_position.y,
                                     prepick_position.z + .23, 10)) {
    RCLCPP_ERROR(LOGGER, "Retreat kinematics plan failed!");
    robot_->move_group_robot_->stop();
    robot_->clearOrientationConstraints();
    robot_->move_group_robot_->setPlanningTime(20.0); // Default for next
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return BT::NodeStatus::FAILURE;
  }

  robot_->clearOrientationConstraints();
  robot_->move_group_robot_->setPlanningTime(20.0); // Default
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return BT::NodeStatus::SUCCESS;
}