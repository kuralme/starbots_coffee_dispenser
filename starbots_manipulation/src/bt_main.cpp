#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "observer.hpp"
#include "bt_nodes/validate_detection_node.hpp"
#include "bt_nodes/prepick_node.hpp"
#include "bt_nodes/pick_node.hpp"
#include "bt_nodes/preplace_node.hpp"
#include "bt_nodes/place_node.hpp"
#include "bt_nodes/putback_node.hpp"
#include "bt_nodes/return_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto robot_node = std::make_shared<PickAndPlace>();
    auto btros_node = rclcpp::Node::make_shared("bt_main_node");

    BT::BehaviorTreeFactory factory;

    // Register custom BT nodes
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
    factory.registerBuilder<PutBack>("PutBack",
                                     [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                     {
                                         return std::make_unique<PutBack>(name, config, robot_node.get());
                                     });
    factory.registerBuilder<Return>("Return",
                                    [&robot_node](const std::string &name, const BT::NodeConfig &config)
                                    {
                                        return std::make_unique<Return>(name, config, robot_node.get());
                                    });

    // Load BT from XML file
    std::string bt_config_dir = ament_index_cpp::get_package_share_directory("starbots_manipulation") + "/bt_config/";
    std::string tree_file = bt_config_dir + "/coffee_delivery_tree.xml";
    auto tree = factory.createTreeFromFile(tree_file);
    BT::Groot2Publisher publisher(tree);
    // Extract the tree model for Groot2
    // std::string xml_models = BT::writeTreeNodesModelXML(factory);
    // std::string output_path = bt_config_dir + "/delivery_tree_nodes_model.xml";
    // std::ofstream out(output_path);
    // out << xml_models;
    // out.close();

    // Service callback triggers BT execution
    auto service = btros_node->create_service<DeliverCup>(
        "/deliver_coffee",
        [&tree, &robot_node](const std::shared_ptr<DeliverCup::Request> req,
                             std::shared_ptr<DeliverCup::Response> res)
        {
            auto blackboard = tree.rootBlackboard();
            blackboard->set("request", req);

            BT::NodeStatus status = tree.tickWhileRunning();
            res->result = (status == BT::NodeStatus::SUCCESS) ? "Coffee Delivery successful" : "Coffee Delivery failed";
        });

    // Set up observer to publish BT state changes
    auto state_pub = btros_node->create_publisher<std_msgs::msg::String>("/bt_node_status", 10);
    auto observer = std::make_shared<BTStateObserver>(tree, btros_node, state_pub);

    executor.add_node(robot_node);
    executor.add_node(btros_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}