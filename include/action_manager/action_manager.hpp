#include "action_manager/parser.hpp"
#include "action_manager/trajectory_structs.hpp"
#include <map>
#include <string>

namespace booster_action_manager {


class ActionManager
{

public:
    void load_all_actions(const std::string & path);
    ActionTrajectory get_action_data_by_name(const std::string & action_name);
    ActionTrajectory get_action_data_from_json(const std::string & json_string);
    std::vector<std::string> get_action_name_list();

private:
    Parser parser;
    std::vector<std::string> action_name_list;
    std::map<std::string, ActionTrajectory> action_list;

};

} //namespace booster_action_manager