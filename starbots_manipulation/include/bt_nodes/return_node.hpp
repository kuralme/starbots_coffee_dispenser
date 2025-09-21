#pragma once

#include <behaviortree_cpp/action_node.h>
#include "pick_and_place.hpp"

class Return : public BT::SyncActionNode
{
public:
    Return(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    PickAndPlace *robot_;
};