#include "bt_nodes/preplace_node.hpp"

PrePlace::PrePlace(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PrePlace::providedPorts() { return {}; }

BT::NodeStatus PrePlace::tick()
{
    const auto blackboard_ = config().blackboard;
    const auto request = blackboard_->get<std::shared_ptr<DeliverCup::Request>>("request");
    auto preplace_position = robot_->goal_poses_[request->goal_cup_holder];
    preplace_position.z += .55;

    robot_->createOrientationConstraint();
    robot_->ensureElbowUp(); // Elbow-up for better planning when reaching close targets
    robot_->publishStatus("Going to the Pre-place Pose", "INFO", preplace_position);

    // Timeout to avoid hanging indefinitely
    auto future_result = std::async(std::launch::async, [&]()
                                    { return robot_->executeKinematicsPlan(preplace_position.x, preplace_position.y, preplace_position.z); });
    if (future_result.wait_for(std::chrono::seconds(40)) != std::future_status::ready || !future_result.get())
    {
        RCLCPP_ERROR(LOGGER, "PrePlace: Kinematics plan failed or timed out.");
        robot_->move_group_robot_->stop();
        robot_->clearOrientationConstraints();
        return BT::NodeStatus::FAILURE;
    }
    robot_->clearOrientationConstraints();
    return BT::NodeStatus::SUCCESS;
}
