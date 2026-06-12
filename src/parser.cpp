#include "action_manager/parser.hpp"

std::map<std::string, ActionTrajectory> Parser::get_action_list_from_dir(const std::string & path) 
{
    std::map<std::string, ActionTrajectory> action_list; 

    for (const auto & entry : std::filesystem::directory_iterator(path)) {
        std::string name = "";
        std::string file_name = entry.path();
        std::string extension_json = ".json";

        for (int i = path.length(); i < file_name.length() - extension_json.length(); i++) {
            name += file_name[i];
        }

        try {
            std::ifstream file(file_name);
            nlohmann::json action_data = nlohmann::json::parse(file);
            ActionTrajectory action = parse_action_json(action_data, name);
            action_list.insert({name, action});

        } catch (nlohmann::json::parse_error & ex) {
            std::cerr << "failed to load action: " << name << std::endl;
            std::cerr << "parse error at byte: " << ex.byte << std::endl;
        throw ex;
        }
    }

    return action_list;
}

ActionTrajectory Parser::parse_string_json(const std::string & action_json)
{
    try {
        nlohmann::json parsed = nlohmann::json::parse(action_json);

        if (parsed.empty() || !parsed.is_object()) {
            throw std::invalid_argument("JSON must be a non-empty object");
        }

        auto it = parsed.begin();
        const std::string action_name = it.key();
        const nlohmann::json & action_data = it.value();

        return parse_action_json(action_data, action_name);

    } catch (const nlohmann::json::parse_error & e) {
        throw std::invalid_argument(std::string("Invalid JSON string: ") + e.what());
    }
}

ActionTrajectory Parser::parse_action_json(const nlohmann::json & action_data, const std::string & action_name)
{
    ActionTrajectory new_action;
    new_action.action_name = action_name;
    new_action.play_in_mode = action_data["play_in_mode"].get<std::string>();
    new_action.next_mode = action_data["next"].get<std::string>();

    std::vector<std::string> joint_names;
    joint_names.reserve(Joint::kJointCnt);
    for (const auto & joint_index : Joint::kAllJoints) {
        joint_names.push_back(std::string(Joint::kJointNames.at(joint_index)));
    }
    new_action.trajectory.joint_names = joint_names;

    double accumulated_time = 0.0;

    for (const auto & pose : action_data["poses"]) {
        accumulated_time += pose["time"].get<double>();

        std::vector<double> positions;
        positions.reserve(Joint::kJointCnt);
        for (const auto & joint_index : Joint::kAllJoints) {
            const std::string joint_name = std::string(Joint::kJointNames.at(joint_index));
            positions.push_back(pose["joints"].value(joint_name, 0.0));
        }

        std::vector<double> velocities(Joint::kJointCnt, 0.0);

        int32_t sec = static_cast<int32_t>(accumulated_time);
        uint32_t nanosec = static_cast<uint32_t>((accumulated_time - sec) * 1e9);

        TrajectoryPoint point;
        point.positions = positions;
        point.velocities = velocities;
        point.delay_before_seconds = pose["delay_before"].get<double>();
        point.duration_seconds = pose["duration_seconds"].get<double>();

        new_action.trajectory.points.push_back(point);
    }

    return new_action;
}
