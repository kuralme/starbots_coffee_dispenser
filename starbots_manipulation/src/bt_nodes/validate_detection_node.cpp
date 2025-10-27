#include "bt_nodes/validate_detection_node.hpp"

ValidateDetection::ValidateDetection(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList ValidateDetection::providedPorts() { return {}; }

BT::NodeStatus ValidateDetection::tick()
{
    int no_detection_count = 0;
    const int max_attempts = 5;
    robot_->publishStatus("Checking detection...", "INFO");

    while (!robot_->goal_poses_received_ ||
           !robot_->obj_pose_received_ ||
           robot_->obj_position_.y * robot_->obj_height_ * robot_->obj_radius_ == 0.0)
    {
        RCLCPP_WARN(LOGGER, "Detection data not received yet (attempt %d/%d)", no_detection_count + 1, max_attempts);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        no_detection_count++;

        if (no_detection_count >= max_attempts)
        {
            robot_->publishStatus("Detection timeout after the attempts", "ERROR");
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::SUCCESS;
}