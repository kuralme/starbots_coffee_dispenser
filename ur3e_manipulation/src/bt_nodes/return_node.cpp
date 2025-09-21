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

    return BT::NodeStatus::SUCCESS;
}