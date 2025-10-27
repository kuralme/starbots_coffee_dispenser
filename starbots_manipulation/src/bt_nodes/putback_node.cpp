#include "bt_nodes/putback_node.hpp"

PutBack::PutBack(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot)
    : BT::SyncActionNode(name, config), robot_(robot) {}
BT::PortsList PutBack::providedPorts() { return {}; }

BT::NodeStatus PutBack::tick()
{
    auto blackboard_ = config().blackboard;
    blackboard_->set("place_failed", true);

    auto prepick_pos = robot_->obj_position_;
    prepick_pos.z += .3;

    robot_->publishStatus("Placement failed. Putting the cup back to original position", "WARN");

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

    return BT::NodeStatus::SUCCESS;
}