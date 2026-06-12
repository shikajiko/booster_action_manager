#include "action_manager/node/action_manager_node.hpp"

namespace booster_action_manager  // add this
{

ActionManagerNode::ActionManagerNode(rclcpp::Node::SharedPtr node)
    : node(node)
{
    trajectory_client = std::make_shared<TrajectoryClient>(node);

    run_action_subscriber = node->create_subscription<action_interface::msg::RunAction>(
        "/run_action",
        10,
        [this](const action_interface::msg::RunAction::SharedPtr msg) {
            this->handle_action_request(msg);
        }
    );
}

void ActionManagerNode::handle_action_request(const action_interface::msg::RunAction::SharedPtr msg)
{
    if (msg->json_data.empty()) {
        ActionTrajectory action_to_play = action_manager.get_action_data_by_name(msg->action_name);
        trajectory_client->send_goal(action_to_play);
    } else {
        ActionTrajectory action_to_play = parser.parse_string_json(msg->json_data);
        trajectory_client->send_goal(action_to_play);
    }
}

} // namespace booster_action_manager  