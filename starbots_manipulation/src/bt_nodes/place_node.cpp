#include "bt_nodes/place_node.hpp"

Place::Place(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Place::providedPorts() { return {}; }

BT::NodeStatus Place::tick()
{
    robot_->publishStatus("Approaching to place...", "INFO");
    geometry_msgs::msg::Pose robot_pose = robot_->move_group_robot_->getCurrentPose().pose;

    robot_->createOrientationConstraint();
    robot_->move_group_robot_->setPlanningTime(3.0);
    if (!robot_->executeKinematicsPlan(robot_pose.position.x, robot_pose.position.y,
                                       robot_pose.position.z - 0.23, 8))
    {
        robot_->publishStatus("Attempts to place failed!", "ERROR");
        robot_->defaultPlanningSettings();
        return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    robot_->executeGripperPlan("gripper_open");
    robot_->detachObject();
    robot_->publishStatus("Cup is placed", "INFO");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    RCLCPP_INFO(LOGGER, "Retreating...");
    if (!robot_->executeKinematicsPlan(robot_pose.position.x, robot_pose.position.y,
                                       robot_pose.position.z, 5))
    {
        RCLCPP_ERROR(LOGGER, "Retreat kinematics plan failed!");
        robot_->defaultPlanningSettings();
        return BT::NodeStatus::FAILURE;
    }

    robot_->defaultPlanningSettings();
    return BT::NodeStatus::SUCCESS;
}