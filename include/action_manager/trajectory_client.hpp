#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interface/action/action_trajectory.hpp"
#include "action_interface/msg/joint_trajectory_point.hpp"
#include "action_manager/trajectory_structs.hpp"


namespace booster_action_manager 
{


using ActionTrajectoryRos = action_interface::action::ActionTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<ActionTrajectoryRos>;


class TrajectoryClient {

public:
    explicit TrajectoryClient(rclcpp::Node::SharedPtr node);
    void send_goal(const ActionTrajectory & action);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp_action::Client<ActionTrajectoryRos>::SharedPtr client;
};

} // namespace booster_action_manager