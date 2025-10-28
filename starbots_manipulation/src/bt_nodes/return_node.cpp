#include "bt_nodes/return_node.hpp"

Return::Return(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Return::providedPorts() { return {}; }

BT::NodeStatus Return::tick()
{
    robot_->publishStatus("Returning to beginning position...", "INFO");
    robot_->obj_position_ = geometry_msgs::msg::Point();
    robot_->obj_height_ = robot_->obj_radius_ = robot_->obj_thickness_ = 0.0;
    robot_->obj_pose_received_ = false;

    robot_->goal_poses_.clear();
    if (!robot_->gotoPredefined("quick_pick"))
    {
        robot_->move_group_robot_->stop();
        robot_->clearOrientationConstraints();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        robot_->gotoPredefined("home");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        robot_->gotoPredefined("quick_pick");
    }
    robot_->publishStatus("Done.", "INFO");

    const auto blackboard_ = config().blackboard;
    auto place_failed = blackboard_->get<bool>("place_failed");
    return place_failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}