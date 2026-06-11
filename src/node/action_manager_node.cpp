#include "rclcpp/rclcpp.hpp"
#include "action_manager/action_manager.hpp"
#include "action_interface/msg/run_action.hpp"
#include "action_manager/trajectory_client.hpp"

class ActionManagerNode 
{
public:
    explicit ActionManagerNode(rclcpp::Node SharedPtr & node);
    void handle_action_request(const action_interface::msg::RunAction & msg);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<action_interface::msg::RunAction> run_action_subscriber;
    ActionManager action_manager;
    std::shared_ptr<TrajectoryClient> trajectory_client;
}

ActionManagerNode::ActionManagerNode(rclcpp::Node SharedPtr & node) : node();
{
    run_action_subscriber = node->create_subscription<action_interface::msg::RunAction>{
        "/run_action",
        10,
        [this](const action_interface::msg::RunAction & msg) {
            handle_action_request(msg);
        }
    }
}

ActionManagerNode::handle_action_request(const action_interface::msg::RunAction & msg) 
{
    if (msg->json_data.empty()) {
        // TO DO: get the correct action data from action manager 
        // TO DO: call send_goal with the obtained action data
    } else {
        // TO DO: convert json data into action data
        // TO DO: call send_goal with the obtained action data
    }
}