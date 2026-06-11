#include "rclcpp/rclcpp.hpp"
#include "action_manager/action_manager.hpp"

class ActionManagerNode 
{
public:
    explicit ActionManagerNode(rclcpp::Node SharedPtr node);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<> run_action_subscriber;


}