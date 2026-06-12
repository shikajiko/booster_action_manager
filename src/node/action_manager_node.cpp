#include "action_manager/node/action_manager_node.hpp"

ActionManagerNode::ActionManagerNode(rclcpp::Node SharedPtr & node) : node();
{
    run_action_subscriber = node->create_subscription<action_interface::msg::RunAction>{
        "/run_action",
        10,
        [this](const action_interface::msg::RunAction & msg) {
            this->handle_action_request(msg);
        }
    }
}

ActionManagerNode::handle_action_request(const action_interface::msg::RunAction & msg) 
{
    if (msg->json_data.empty()) {
        ActionTrajectory action_to_play = action_manager.get_action_data_by_name(msg->action_name);
        trajectory_client->send_goal(action_to_play);
        // TO DO: call send_goal with the obtained action data
    } else {
        // TO DO: convert json data into action data
        // TO DO: call send_goal with the obtained action data
    }
}