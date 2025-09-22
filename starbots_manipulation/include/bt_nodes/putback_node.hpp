#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/tree_node.h>
#include "pick_and_place.hpp"

class PutBack : public BT::SyncActionNode
{
public:
    PutBack(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    PickAndPlace *robot_;
};
