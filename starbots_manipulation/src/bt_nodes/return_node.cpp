#include "bt_nodes/return_node.hpp"

Return::Return(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Return::providedPorts() { return {}; }

BT::NodeStatus Return::tick()
{
    robot_->obj_position_ = geometry_msgs::msg::Point();
    robot_->obj_height_ = robot_->obj_radius_ = robot_->obj_thickness_ = 0.0;
    robot_->goal_poses_ = std::vector<geometry_msgs::msg::Point>();
    robot_->obj_pose_received_ = false;
    robot_->goal_poses_received_ = false;
    robot_->gotoPredefined("quick_pick");

    bool place_failed = false;
    auto blackboard_ = config().blackboard;
    // blackboard_->get("place_failed", place_failed);
    if (blackboard_->get("place_failed", place_failed))
    {
        RCLCPP_INFO(LOGGER, "Place failed: %s", place_failed ? "TRUE" : "FALSE");
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Failed to get place_failed from blackboard.");
    }
    return place_failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}