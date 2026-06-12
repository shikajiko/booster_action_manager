#include "action_manager/trajectory_client.hpp"

namespace booster_action_manager
{

TrajectoryClient::TrajectoryClient(rclcpp::Node::SharedPtr node)
    : node(node)  
{
    client = rclcpp_action::create_client<ActionTrajectoryRos>(
        node,                        
        "controller/run_trajectory");
}

void TrajectoryClient::send_goal(const ActionTrajectory & action)
{
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available");
        return;
    }

    auto goal_msg = ActionTrajectoryRos::Goal();
    goal_msg.trajectory.joint_names = action.trajectory.joint_names;
    
    for (const auto & point : action.trajectory.points) {  
        action_interface::msg::JointTrajectoryPoint ros_point; ros_point;
        ros_point.positions = point.positions;
        ros_point.duration_seconds = point.duration_seconds;
        ros_point.delay_before_seconds = point.delay_before_seconds;
        goal_msg.trajectory.points.push_back(ros_point);
    }

    auto send_goal_options = rclcpp_action::Client<ActionTrajectoryRos>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandle::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(node->get_logger(), "Goal rejected");
            } else {
                RCLCPP_INFO(node->get_logger(), "Goal accepted");
            }
        };  

    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr, const std::shared_ptr<const ActionTrajectoryRos::Feedback> feedback) {
            (void)feedback;
        };

    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node->get_logger(), "Goal succeeded");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Goal failed");
            }
        };

    client->async_send_goal(goal_msg, send_goal_options);
}

} // namespace booster_action_manager