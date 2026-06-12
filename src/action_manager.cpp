#include "action_manager/action_manager.hpp"

namespace booster_action_manager {


ActionTrajectory ActionManager::get_action_data_by_name(const std::string & action_name) 
{
    return action_list[action_name];
}

std::vector<std::string> ActionManager::get_action_name_list()
{
    return action_name_list;
}

void ActionManager::load_all_actions(const std::string & path) 
{
    action_list = parser.get_action_list_from_dir(path);
}

ActionTrajectory ActionManager::get_action_data_from_json(const std::string & json_string) 
{
    return parser.parse_string_json(json_string);
}

} // namespace booster_action_manager