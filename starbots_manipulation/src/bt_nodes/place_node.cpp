#include "bt_nodes/place_node.hpp"

Place::Place(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Place::providedPorts() { return {}; }

BT::NodeStatus Place::tick()
{
    // robot_current_state_ = robot_->move_group_robot_->getCurrentState(10);
    geometry_msgs::msg::Pose robot_pose = robot_->move_group_robot_->getCurrentPose().pose;

    RCLCPP_INFO(LOGGER, "Approaching to place...");

    robot_->createOrientationConstraint();
    robot_->clearOctomap();
    if (!robot_->executeKinematicsPlan(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z - 0.230))
    {
        robot_->executeCartesianPlan(+0.000, +0.000, -0.250);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    robot_->executeGripperPlan("gripper_open");
    robot_->detachObject();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    RCLCPP_INFO(LOGGER, "Retreating...");
    if (!robot_->executeKinematicsPlan(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z))
    {
        robot_->executeCartesianPlan(+0.000, +0.000, +0.300);
    }

    robot_->clearOrientationConstraints();
    return BT::NodeStatus::SUCCESS;
}