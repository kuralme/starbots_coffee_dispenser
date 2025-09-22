#pragma once

#include <behaviortree_cpp/loggers/bt_observer.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class BTStateObserver : public BT::TreeObserver
{
public:
    BTStateObserver(const BT::Tree &tree,
                    rclcpp::Node::SharedPtr node,
                    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);

    void callback(BT::Duration timestamp,
                  const BT::TreeNode &node,
                  BT::NodeStatus prev_status,
                  BT::NodeStatus status) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};