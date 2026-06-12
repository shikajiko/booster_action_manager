#ifndef BOOSTER_ACTION_MANAGER_TRAJECTORY_STRUCTS_HPP
#define BOOSTER_ACTION_MANAGER_TRAJECTORY_STRUCTS_HPP

#include <string>
#include <vector>

namespace booster_action_manager 
{
struct TrajectoryPoint {
    std::vector<double> positions;
    std::vector<double> velocities;
    double duration_seconds;
    double delay_before_seconds = 0.0;
};

struct Trajectory {
    std::vector<std::string> joint_names;
    std::vector<TrajectoryPoint> points;
};

struct ActionTrajectory {
    std::string action_name;
    std::string play_in_mode;
    std::string next_mode;
    Trajectory trajectory;
};

} // namespace booster_action_manager

#endif
