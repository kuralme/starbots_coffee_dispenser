#include "observer.hpp"

BTStateObserver::BTStateObserver(const BT::Tree &tree,
                                 rclcpp::Node::SharedPtr node,
                                 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
    : BT::TreeObserver(tree), node_(node), publisher_(publisher) {}

void BTStateObserver::callback(BT::Duration timestamp,
                               const BT::TreeNode &node,
                               BT::NodeStatus prev_status,
                               BT::NodeStatus status)
{
    if (prev_status != status)
    {
        std::string log_output = "[BT] Node '" + node.name() + "' changed from " +
                                 BT::toStr(prev_status, true) + " to " +
                                 BT::toStr(status, true);
        RCLCPP_INFO(node_->get_logger(), "%s", log_output.c_str());

        // Only publish status for leaf nodes
        if (prev_status != status)
        {
            std_msgs::msg::String msg;
            msg.data = "[BT] " + node.name() + " Node -> " + BT::toStr(status, false);
            publisher_->publish(msg);
        }
    }
};