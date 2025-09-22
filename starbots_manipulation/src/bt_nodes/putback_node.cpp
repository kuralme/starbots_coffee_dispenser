#include "bt_nodes/putback_node.hpp"

PutBack::PutBack(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PutBack::providedPorts()
{
    return {BT::InputPort<bool>("place_failed")};
}

BT::NodeStatus PutBack::tick()
{
    auto prepick_pos = robot_->obj_position_;
    prepick_pos.z += .3;
    RCLCPP_INFO(LOGGER, "Putting the cup back to original position");

    robot_->createOrientationConstraint();
    robot_->move_group_robot_->clearPoseTargets();
    robot_->executeKinematicsPlan(prepick_pos.x, prepick_pos.y, prepick_pos.z);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    robot_->executeCartesianPlan(+0.000, +0.000, -0.079);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    robot_->executeGripperPlan("gripper_open");
    robot_->detachObject();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    robot_->executeCartesianPlan(+0.000, +0.000, +0.079);
    robot_->clearOrientationConstraints();

    auto blackboard_ = config().blackboard;
    blackboard_->set("place_failed", true);
    return BT::NodeStatus::FAILURE;
}