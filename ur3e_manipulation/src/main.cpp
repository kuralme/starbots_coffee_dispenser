#include "starbots_delivery_server.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pick_and_place_as = std::make_shared<PickAndPlace>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pick_and_place_as);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}