#include "bt_nodes/validate_detection_node.hpp"

ValidateDetection::ValidateDetection(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList ValidateDetection::providedPorts() { return {}; }

BT::NodeStatus ValidateDetection::tick()
{
    int no_detection_count = 0;
    const int max_attempts = 5;
    const std::chrono::milliseconds wait_time(3000);

    while (!robot_->goal_poses_received_ ||
           !robot_->obj_pose_received_ ||
           robot_->obj_position_.y * robot_->obj_height_ * robot_->obj_radius_ == 0.0)
    {
        RCLCPP_WARN(LOGGER, "Detection data not received yet (attempt %d/%d)", no_detection_count + 1, max_attempts);
        std::this_thread::sleep_for(wait_time);
        no_detection_count++;

        if (no_detection_count >= max_attempts)
        {
            RCLCPP_ERROR(LOGGER, "Detection timeout after %d attempts", max_attempts);
            return BT::NodeStatus::FAILURE;
        }
    }

    // Reset the place_failed flag for consecutive attempts
    auto blackboard_ = config().blackboard;
    blackboard_->set("place_failed", false);
    return BT::NodeStatus::SUCCESS;
}