#include "action_manager/change_mode_client.hpp"

namespace booster_action_manager
{
using UpperControl = booster_model::mode::UpperControl;

ChangeModeClient::ChangeModeClient(rclcpp::Node::SharedPtr node) : node(node)
{
    mode_client = node->create_client<SetMode>("client/set_mode");
    upper_control_client = node->create_client<SetUpperControl>("client/set_upper_control");
    robot_state_subscriber =
        node->create_subscription<RobotState>(
            "/robot_states",
            10,
            [this](const RobotState::SharedPtr msg)
            {
                this->update_robot_state(msg);
            }
        );
}

void ChangeModeClient::update_robot_state(RobotState::SharedPtr msg)
{
    current_mode = static_cast<RobotMode>(msg->current_mode);
    upc_status = int_to_upper_control(msg->current_body_control);
}

void ChangeModeClient::switch_to_target_mode(
    const ControlType & control_type,
    std::function<void(bool)> on_done)
{
    switch (control_type) {
        case ControlType::UPPER_BODY: {
            if (current_mode == RobotMode::WALK && upc_status == UpperControl::ON) {
                on_done(true);
                return;
            }
            auto req = std::make_shared<SetMode::Request>();
            req->mode = static_cast<uint8_t>(RobotMode::WALK);
            mode_client->async_send_request(req,
                [this, on_done](rclcpp::Client<SetMode>::SharedFuture future) {
                    if (!future.get()->success) { on_done(false); return; }
                    auto upper_control_req = std::make_shared<SetUpperControl::Request>();
                    upper_control_req->enable = true;
                    upper_control_client->async_send_request(upper_control_req,
                        [on_done](rclcpp::Client<SetUpperControl>::SharedFuture uc_future) {
                            on_done(uc_future.get()->success);
                        });
                });
            break;
        }
        case ControlType::FULL_BODY: {
            if (current_mode == RobotMode::CUSTOM) {
                on_done(true);
                return;
            }
            auto req = std::make_shared<SetMode::Request>();
            req->mode = static_cast<uint8_t>(RobotMode::CUSTOM);
            mode_client->async_send_request(req,
                [on_done](rclcpp::Client<SetMode>::SharedFuture future) {
                    on_done(future.get()->success);
                });
            break;
        }
        default:
            on_done(true);
            break;
    }
}

UpperControl ChangeModeClient::int_to_upper_control(const int & ctrl)
{
    if (ctrl == 3) {
        return UpperControl::ON;
    }
    return UpperControl::OFF;
}

}