#include "bt_nodes/prepick_node.hpp"

PrePick::PrePick(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PrePick::providedPorts() { return {}; }

BT::NodeStatus PrePick::tick()
{
    auto prepick_pos = robot_->obj_position_;
    prepick_pos.z += .3;
    robot_->publishStatus("Going to the Pre-pick Pose", "INFO", prepick_pos);

    // Timeout after 20 seconds to avoid hanging indefinitely
    auto future_result = std::async(std::launch::async, [&]()
                                    { return robot_->executeKinematicsPlan(prepick_pos.x, prepick_pos.y, prepick_pos.z); });
    if (future_result.wait_for(std::chrono::seconds(20)) != std::future_status::ready || !future_result.get())
    {
        robot_->publishStatus("Attempt to reach prepick failed or timed out!",
                              "ERROR");
        robot_->move_group_robot_->stop();
        return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return BT::NodeStatus::SUCCESS;
}