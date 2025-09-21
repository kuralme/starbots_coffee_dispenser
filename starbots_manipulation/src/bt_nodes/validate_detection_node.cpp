#include "bt_nodes/validate_detection_node.hpp"

ValidateDetection::ValidateDetection(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList ValidateDetection::providedPorts() { return {}; }

BT::NodeStatus ValidateDetection::tick()
{
    while (!robot_->obj_pose_received_ || robot_->obj_position_.y * robot_->obj_height_ * robot_->obj_radius_ == 0.0)
    {
        RCLCPP_WARN(LOGGER, "Cup Pose not received yet!");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    while (!robot_->goal_poses_received_)
    {
        RCLCPP_WARN(LOGGER, "Cup Holder Poses not received yet!");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    return BT::NodeStatus::SUCCESS;
}