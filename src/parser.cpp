#include "action_manager/parser.hpp"
#include <numbers>

namespace booster_action_manager 
{

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
    constexpr double kDegToRad = M_PI / 180.0; 
    
    ActionTrajectory new_action;
    new_action.action_name = action_name;
    std::string control_type = action_data["control_type"].get<std::string>();
    new_action.control_type = string_to_control_type(action_data["control_type"].get<std::string>());
    new_action.next_mode = action_data["next"].get<std::string>();

    std::vector<std::string> joint_names;
    joint_names.reserve(Joint::kJointCnt);
    for (const auto & joint_index : Joint::kAllJoints) {
        joint_names.push_back(std::string(Joint::kJointNames.at(joint_index)));
    }
    new_action.trajectory.joint_names = joint_names;

    for (const auto & pose : action_data["poses"]) {
        std::vector<double> positions;
        positions.reserve(Joint::kJointCnt);
        for (const auto & joint_index : Joint::kAllJoints) {
            const std::string joint_name = std::string(Joint::kJointNames.at(joint_index));
            const double deg = pose["joints"].value(joint_name, 0.0);
            positions.push_back(deg * kDegToRad);
        }

        std::vector<double> velocities(Joint::kJointCnt, 0.0);

        TrajectoryPoint point;
        point.positions = positions;
        point.velocities = velocities;
        point.delay_before_seconds = pose["delay_before"].get<double>();
        point.duration_seconds = pose["duration_seconds"].get<double>();

        new_action.trajectory.points.push_back(point);
    }

    return new_action;
}

ControlType Parser::string_to_control_type(const std::string& str)
{
    if (str == "upper_body")
        return ControlType::UPPER_BODY;

    if (str == "full_body")
        return ControlType::FULL_BODY;

    throw std::runtime_error(
        "Unknown control_type: " + str);
}

} // namespace booster_action_manager