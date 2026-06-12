#ifndef BOOSTER_ACTION_MANAGER_ACTION_MANAGER_NODE_HPP
#define BOOSTER_ACTION_MANAGER_ACTION_MANAGER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "action_manager/action_manager.hpp"
#include "action_interface/msg/run_action.hpp"
#include "action_manager/trajectory_client.hpp"
#include "action_manager/trajectory_structs.hpp"

namespace booster_action_manager 
{

class ActionManagerNode 
{
public:
    explicit ActionManagerNode(rclcpp::Node::SharedPtr node);
    void handle_action_request(const action_interface::msg::RunAction::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<action_interface::msg::RunAction>::SharedPtr run_action_subscriber;
    ActionManager action_manager;
    std::shared_ptr<TrajectoryClient> trajectory_client;
};

} // namespace booster_action_manager  
#endif