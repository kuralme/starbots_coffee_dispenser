#include "bt_nodes/prepick_node.hpp"

PrePick::PrePick(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PrePick::providedPorts() { return {}; }

BT::NodeStatus PrePick::tick()
{
    auto prepick_pos = robot_->obj_position_;
    prepick_pos.z += .3;
    RCLCPP_INFO(LOGGER, "Going to Pre-pick Pose: [%.3f, %.3f, %.3f]",
                prepick_pos.x, prepick_pos.y, prepick_pos.z);

    if (robot_->executeKinematicsPlan(prepick_pos.x, prepick_pos.y, prepick_pos.z))
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Pre-pick failed...");
        return BT::NodeStatus::FAILURE;
    }
}