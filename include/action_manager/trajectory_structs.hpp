#include <string>
#include <vector>

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