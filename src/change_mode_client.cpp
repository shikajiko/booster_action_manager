#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "booster_client_interface/srv/set_mode.hpp"
#include "booster_client_interface/srv/set_upper_control.hpp"
#include "booster_interface/msg/robot_states_msg.hpp"
#include "action_manager/control_enum.hpp"
#include "booster_model/mode_enum.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_action_manager 
{

using RobotMode = booster_model::mode::RobotMode;
using UpperControl = booster_model::mode::UpperControl;
using SetMode = booster_client_interface::srv::SetMode;
using SetUpperControl = booster_client_interface::srv::SetUpperControl;
using RobotState = booster_interface::msg::RobotStatesMsg;

class ChangeModeClient
{
public:
    explicit ChangeModeClient(rclcpp::Node::SharedPtr node);
    void switch_to_target_mode(const ControlType & control_type);
    void update_robot_state(RobotState::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Client<SetMode>::SharedPtr mode_client;
    rclcpp::Client<SetUpperControl>::SharedPtr upper_control_client;
    rclcpp::Subscriber<RobotState>::SharedPtr robot_state_subscriber;

    RobotMode current_mode;
    UpperControl upc_status;
}

ChangeModeClient::ChangeModeClient(rclcpp::Node::SharedPtr node) : node(node)
{
    mode_client = create_client<SetMode>("client/set_mode");
    upper_control_client = create_client<SetUpperControl>("client/set_upper_control");

    robot_state_subscriber = 
        node->create_subscriber<RobotState>(
            "/robot_states",
            10,
            [this](const RobotState::SharedPtr msg) 
            {
                this->update_robot_state(msg);
            }
        )
}

ChangeModeClient::update_mode(RobotState::SharedPtr msg) 
{
    this.current_mode = static_cast<RobotMode>(msg->current_mode);
    this.upc_status = int_to_upper_control(msg->current_body_control);
}

ChangeModeClient::switch_to_correct_mode(const ControlType & control_type) 
{
    if (robot_state == RobotMode::CUSTOM) {
        return;
    }

    switch (control_type)
    {
    case ControlType::UPPER_BODY:
        // change to walk with upc on
        /* code */
        break;
    
    case ControlType::FULL_BODY:
        // change to custom
        break;

    default:
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