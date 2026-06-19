#include "action_manager/node/action_manager_node.hpp"

namespace booster_action_manager  // add this
{

ActionManagerNode::ActionManagerNode(rclcpp::Node::SharedPtr node)
    : node(node)  
{
    trajectory_client = std::make_shared<TrajectoryClient>(node);
    change_mode_client = std::make_shared<ChangeModeClient>(node);

    run_action_subscriber = node->create_subscription<action_interface::msg::RunAction>(
        "/run_action",
        10,
        [this](const action_interface::msg::RunAction::SharedPtr msg) {
            this->handle_action_request(msg);
        }
    );
}

void ActionManagerNode::handle_action_request(
    const action_interface::msg::RunAction::SharedPtr msg)
{
    ControlType control_type;
    ActionTrajectory action_to_play = msg->json_data.empty()
        ? action_manager.get_action_data_by_name(msg->action_name, control_type)
        : action_manager.get_action_data_from_json(msg->json_data, control_type);

    change_mode_client->switch_to_target_mode(control_type,
        [this, action_to_play](bool success) {
            if (!success) {
                RCLCPP_WARN(node->get_logger(), "Mode switch failed, dropping trajectory");
                return;
            }
            trajectory_client->send_goal(action_to_play);  
        });
}

} // namespace booster_action_manager  