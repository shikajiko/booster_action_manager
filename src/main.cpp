#include "action_manager/node/action_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<rclcpp::Node>("booster_action_manager");
    const booster_action_manager::ActionManagerNode action_manager_node(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}