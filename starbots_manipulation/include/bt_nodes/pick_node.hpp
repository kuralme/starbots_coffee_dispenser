#pragma once

#include <behaviortree_cpp/action_node.h>
#include "pick_and_place.hpp"

class Pick : public BT::SyncActionNode
{
public:
    Pick(const std::string &name, const BT::NodeConfig &config, PickAndPlace *robot);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    PickAndPlace *robot_;
};