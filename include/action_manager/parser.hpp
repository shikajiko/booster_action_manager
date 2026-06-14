#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include "booster_model/joint.hpp"
#include "booster_model/joint_map.hpp"
#include "action_manager/trajectory_structs.hpp"
#include "action_manager/control_enum.hpp"
namespace booster_action_manager 
{

namespace Joint = booster_model::Joint;

class Parser
{
public:
    std::map<std::string, ActionTrajectory> get_action_list_from_dir(const std::string & path);
    ActionTrajectory parse_action_json(const nlohmann::json & action_data, const std::string & action_name);
    ActionTrajectory parse_string_json(const std::string & action_json);
private: 
    ControlType string_to_control_type(const std::string& str);
};

} // namespace booster_action_manager
