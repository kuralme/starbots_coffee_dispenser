#include "bt_nodes/pick_node.hpp"

Pick::Pick(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList Pick::providedPorts() { return {}; }

BT::NodeStatus Pick::tick()
{
    RCLCPP_INFO(LOGGER, "Approaching to pick...");
    robot_->executeGripperPlan("gripper_open");
    robot_->clearOctomap();
    robot_->executeCartesianPlan(+0.000, +0.000, -0.079);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    robot_->closeGripperIncremental();
    robot_->attachObject();
    robot_->executeCartesianPlan(+0.000, +0.000, +0.079);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return BT::NodeStatus::SUCCESS;
}