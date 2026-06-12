#include "rclcpp/rclcpp.hpp"
#include "action_interface/action/action_trajectory.hpp"
#include "action_manager/trajectory_structs.hpp"


namespace booster_action_manager 
{


using ActionTrajectoryRos = action_interface::action:ActionTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<ActionTrajectoryRos>;


class TrajectoryClient {

public:
    explicit TrajectoryClient(rclcpp::Node node);
    void send_goal(const ActionTrajectory & action);

private:
    rclcpp::Node node;
    rclcpp_action<ActionTrajectoryRos> client;
};

} // namespace booster_action_manager