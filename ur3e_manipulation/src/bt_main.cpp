#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include "bt_nodes/validate_detection_node.hpp"
#include "bt_nodes/prepick_node.hpp"
#include "bt_nodes/pick_node.hpp"
#include "bt_nodes/preplace_node.hpp"
#include "bt_nodes/place_node.hpp"
#include "bt_nodes/return_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto robot_node = std::make_shared<PickAndPlace>();
    auto btros_node = rclcpp::Node::make_shared("bt_main_node");

    // Register custom BT nodes
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<ValidateDetection>("ValidateDetection",
                                               [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                               {
                                                   return std::make_unique<ValidateDetection>(name, config, robot_node.get());
                                               });
    factory.registerBuilder<Pick>("Pick",
                                  [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                  {
                                      return std::make_unique<Pick>(name, config, robot_node.get());
                                  });
    factory.registerBuilder<PrePick>("PrePick",
                                     [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                     {
                                         return std::make_unique<PrePick>(name, config, robot_node.get());
                                     });
    factory.registerBuilder<PrePlace>("PrePlace",
                                      [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                      {
                                          return std::make_unique<PrePlace>(name, config, robot_node.get());
                                      });
    factory.registerBuilder<Place>("Place",
                                   [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                   {
                                       return std::make_unique<Place>(name, config, robot_node.get());
                                   });
    factory.registerBuilder<Return>("Return",
                                    [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                    {
                                        return std::make_unique<Return>(name, config, robot_node.get());
                                    });

    std::string tree_file = ament_index_cpp::get_package_share_directory("ur3e_manipulation") + "/config/coffee_delivery_tree.xml";
    auto tree = factory.createTreeFromFile(tree_file);

    // Service callback triggers BT execution
    auto service = btros_node->create_service<DeliverCup>(
        "/deliver_coffee",
        [&tree, &robot_node](const std::shared_ptr<DeliverCup::Request> req,
                             std::shared_ptr<DeliverCup::Response> res)
        {
            // Set up blackboard
            auto blackboard = tree.rootBlackboard();
            blackboard->set("request", req);

            BT::NodeStatus status = tree.tickWhileRunning();
            res->result = (status == BT::NodeStatus::SUCCESS) ? "Coffee Delivery successful" : "Coffee Delivery failed";
        });
    RCLCPP_INFO(btros_node->get_logger(), "Coffee delivery service initialized.");

    executor.add_node(robot_node);
    executor.add_node(btros_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}