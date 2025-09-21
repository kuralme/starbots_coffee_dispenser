#include "bt_nodes/preplace_node.hpp"

PrePlace::PrePlace(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PrePlace::providedPorts() { return {}; }

BT::NodeStatus PrePlace::tick()
{
    auto blackboard_ = config().blackboard;
    auto request = blackboard_->get<std::shared_ptr<DeliverCup::Request>>("request");
    auto goal_position = robot_->goal_poses_[request->goal_cup_holder];
    goal_position.z += .55;

    RCLCPP_INFO(LOGGER, "Going to the Pre-placing Pose: [%.3f, %.3f, %.3f]",
                goal_position.x, goal_position.y, goal_position.z);
    if (!robot_->executeKinematicsPlan(goal_position.x, goal_position.y, goal_position.z))
    {
        RCLCPP_WARN(LOGGER, "Delivery failed...");
        robot_->move_group_robot_->clearPoseTargets();
        robot_->executeCartesianPlan(+0.000, +0.000, -0.079);
        robot_->executeGripperPlan("gripper_open");
        return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return BT::NodeStatus::SUCCESS;
}
