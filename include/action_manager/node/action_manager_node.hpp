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
    explicit ActionManagerNode(rclcpp::Node SharedPtr & node);
    void handle_action_request(const action_interface::msg::RunAction & msg);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<action_interface::msg::RunAction> run_action_subscriber;
    ActionManager action_manager;
    std::shared_ptr<TrajectoryClient> trajectory_client;
}

} // namespace booster_action_manager  