#include "bt_nodes/place_node.hpp"

Place::Place(const std::string &name, const BT::NodeConfig &config,
             PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Place::providedPorts() { return {}; }

BT::NodeStatus Place::tick()
{

  const auto blackboard_ = config().blackboard;
  const auto request =
      blackboard_->get<std::shared_ptr<DeliverCup::Request>>("request");
  auto prepick_position = robot_->goal_poses_[request->goal_cup_holder];
  prepick_position.z += .29;

  robot_->publishStatus("Approaching to place...", "INFO");

  robot_->createOrientationConstraint();
  robot_->move_group_robot_->setPlanningTime(5.0);
  if (!robot_->executeKinematicsPlan(prepick_position.x, prepick_position.y,
                                     prepick_position.z, 5))
  {
    robot_->publishStatus("Attempts to place failed!", "ERROR");
    robot_->defaultPlanningSettings();
    return BT::NodeStatus::FAILURE;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  robot_->executeGripperPlan("gripper_open");
  robot_->detachCollisionObject("coffee_cup");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(LOGGER, "Retreating...");
  if (!robot_->executeKinematicsPlan(prepick_position.x, prepick_position.y,
                                     prepick_position.z + .23, 5))
  {
    robot_->publishStatus("Attempts to retreat failed!", "ERROR");
    robot_->defaultPlanningSettings();
    return BT::NodeStatus::FAILURE;
  }

  robot_->defaultPlanningSettings();
  return BT::NodeStatus::SUCCESS;
}