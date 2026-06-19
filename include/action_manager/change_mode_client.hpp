#include "booster_client_interface/srv/set_mode.hpp"
#include "booster_client_interface/srv/set_upper_control.hpp"
#include "booster_interface/msg/robot_states_msg.hpp"
#include "action_manager/control_enum.hpp"
#include "booster_model/mode_enum.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_action_manager 
{
    
class ChangeModeClient
{

public:
    explicit ChangeModeClient(rclcpp::Node::SharedPtr node);
    void switch_to_target_mode(const ControlType & control_type);
    void update_robot_state(RobotState::SharedPtr msg);

private:
    using RobotMode = booster_model::mode::RobotMode;
    using UpperControl = booster_model::mode::UpperControl;
    using SetMode = booster_client_interface::srv::SetMode;
    using SetUpperControl = booster_client_interface::srv::SetUpperControl;
    using RobotState = booster_interface::msg::RobotStatesMsg;

    rclcpp::Node::SharedPtr node;
    rclcpp::Client<SetMode>::SharedPtr mode_client;
    rclcpp::Client<SetUpperControl>::SharedPtr upper_control_client;
    rclcpp::Subscriber<RobotState>::SharedPtr robot_state_subscriber;

    RobotMode current_mode;
    UpperControl upc_status;
}

}