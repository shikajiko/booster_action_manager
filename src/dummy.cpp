
void ActionManager::load_config(const std::string & path)
{
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

      Action action = load_action(action_data, name);

      actions.insert({name, action});
    } catch (nlohmann::json::parse_error & ex) {
      std::cerr << "failed to load action: " << name << std::endl;
      std::cerr << "parse error at byte " << ex.byte << std::endl;
      throw ex;
    }
  }
}

Action ActionManager::load_action(
  const nlohmann::json & action_data,
  const std::string & action_name) const
{
  Action action = Action(action_name);

  action.set_name(action_data["name"]);

  for (const auto & [key, val] : action_data.items()) {
    if (key.find("poses") != std::string::npos) {
      for (const auto & raw_pose : action_data["poses"]) {
        {
          using tachimawari::joint::JointId;
          using tachimawari::joint::Joint;

          Pose pose(raw_pose["name"]);
          std::vector<Joint> joints;

          for (const auto & [joint_key, joint_val] : raw_pose["joints"].items()) {
            if (joint_key.compare(0, 4, "neck") == 0) {
              continue;
            }

            Joint joint(JointId::by_name.at(joint_key), joint_val);
            joints.push_back(joint);
          }

          pose.set_pause(raw_pose["pause"]);
          pose.set_speed(raw_pose["speed"]);
          pose.action_time = raw_pose["time"];
          pose.set_joints(joints);
          action.add_pose(pose);
        }
      }
    } else if (key == "start_delay") {
      action.set_start_delay(val);
    } else if (key == "stop_delay") {
      action.set_stop_delay(val);
    } else if (key == "next") {
      action.set_next_action(val);
    } else if (key == "time_based") {
      action.time_based = val;
    }
  }

  return action;
}

