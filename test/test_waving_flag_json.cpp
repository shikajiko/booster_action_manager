// test_parser_waving_flag.cpp

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "action_manager/parser.hpp"
#include "action_manager/control_enum.hpp"

namespace {

constexpr double kTol = 1e-6;

double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

std::unordered_map<std::string, double> base_joint_degrees() {
    return {
        {"HeadYaw", 0.0},
        {"HeadPitch", 0.0},

        {"LeftShoulderPitch", -70.47},
        {"LeftShoulderRoll",   -93.97},
        {"LeftElbowPitch",      63.58},
        {"LeftElbowYaw",       -49.27},

        {"RightShoulderPitch", -49.27},
        {"RightShoulderRoll",   95.10},
        {"RightElbowPitch",     53.87},
        {"RightElbowYaw",       37.24},

        {"LeftHipPitch", -28.65},
        {"LeftHipRoll",    0.0},
        {"LeftHipYaw",     0.0},
        {"LeftKneePitch",  6.01},
        {"CrankUpLeft",   -4.58},
        {"CrankDownLeft", -4.58},

        {"RightHipPitch", -28.65},
        {"RightHipRoll",    0.0},
        {"RightHipYaw",     0.0},
        {"RightKneePitch",  6.01},
        {"CrankUpRight",  -4.58},
        {"CrankDownRight",-4.58}
    };
}

std::unordered_map<std::string, double> point_1_degrees() {
    auto d = base_joint_degrees();
    d["LeftElbowPitch"]  = 88.86;
    d["LeftElbowYaw"]    = -61.88;
    d["RightShoulderPitch"] = -36.12;
    d["RightElbowPitch"]  = 67.04;
    d["RightElbowYaw"]    = 50.42;
    return d;
}

std::unordered_map<std::string, double> point_2_degrees() {
    auto d = base_joint_degrees();
    d["LeftShoulderPitch"] = -66.47;
    d["LeftShoulderRoll"]   = -71.07;
    d["LeftElbowPitch"]     = 88.86;
    d["LeftElbowYaw"]       = -83.08;
    d["RightShoulderPitch"] = -32.66;
    d["RightShoulderRoll"]   = 93.41;
    d["RightElbowPitch"]     = 58.44;
    d["RightElbowYaw"]       = 45.84;
    return d;
}

std::unordered_map<std::string, double> point_3_degrees() {
    return point_1_degrees();
}

void expect_point_matches_map(
    const booster_action_manager::ActionTrajectory & action,
    std::size_t point_index,
    const std::unordered_map<std::string, double> & expected_deg)
{
    ASSERT_LT(point_index, action.trajectory.points.size());

    const auto & names = action.trajectory.joint_names;
    const auto & point = action.trajectory.points[point_index];

    ASSERT_EQ(names.size(), point.positions.size());

    for (std::size_t i = 0; i < names.size(); ++i) {
        const auto & joint_name = names[i];
        auto it = expected_deg.find(joint_name);
        ASSERT_NE(it, expected_deg.end()) << "Missing expected value for joint: " << joint_name;

        const double expected_rad = deg_to_rad(it->second);
        EXPECT_NEAR(point.positions[i], expected_rad, kTol)
            << "Mismatch at point " << point_index << ", joint " << joint_name;
    }

    EXPECT_DOUBLE_EQ(point.duration_seconds, 2.0);
    EXPECT_DOUBLE_EQ(point.delay_before_seconds, 0.0);
}

}  // namespace

TEST(ParserTest, ParsesWavingFlagJsonIntoExpectedTrajectory)
{
    using booster_action_manager::Parser;
    using booster_action_manager::ControlType;

    // parse_string_json() expects a top-level object where the key is the action name.
    const std::string json = R"JSON(
    {
      "waving_flag": {
        "name": "waving_flag",
        "control_type": "upper_body",
        "next": "",
        "poses": [
          {
            "joints": {
              "HeadPitch": 0.0,
              "LeftShoulderPitch": -70.47,
              "LeftShoulderRoll": -93.97,
              "LeftElbowPitch": 63.58,
              "LeftElbowYaw": -49.27,
              "RightShoulderPitch": -49.27,
              "RightShoulderRoll": 95.10,
              "RightElbowPitch": 53.87,
              "RightElbowYaw": 37.24,
              "LeftHipPitch": -28.65,
              "LeftHipRoll": 0.0,
              "LeftHipYaw": 0.0,
              "LeftKneePitch": 6.01,
              "CrankUpLeft": -4.58,
              "CrankDownLeft": -4.58,
              "RightHipPitch": -28.65,
              "RightHipRoll": 0.0,
              "RightHipYaw": 0.0,
              "RightKneePitch": 6.01,
              "CrankUpRight": -4.58,
              "CrankDownRight": -4.58
            },
            "name": "step_0",
            "delay_before": 0.0,
            "duration": 2.0
          },
          {
            "joints": {
              "HeadPitch": 0.0,
              "LeftShoulderPitch": -70.47,
              "LeftShoulderRoll": -93.97,
              "LeftElbowPitch": 88.86,
              "LeftElbowYaw": -61.88,
              "RightShoulderPitch": -36.12,
              "RightShoulderRoll": 95.10,
              "RightElbowPitch": 67.04,
              "RightElbowYaw": 50.42,
              "LeftHipPitch": -28.65,
              "LeftHipRoll": 0.0,
              "LeftHipYaw": 0.0,
              "LeftKneePitch": 6.01,
              "CrankUpLeft": -4.58,
              "CrankDownLeft": -4.58,
              "RightHipPitch": -28.65,
              "RightHipRoll": 0.0,
              "RightHipYaw": 0.0,
              "RightKneePitch": 6.01,
              "CrankUpRight": -4.58,
              "CrankDownRight": -4.58
            },
            "name": "step_1",
            "delay_before": 0.0,
            "duration": 2.0
          },
          {
            "joints": {
              "HeadPitch": 0.0,
              "LeftShoulderPitch": -66.47,
              "LeftShoulderRoll": -71.07,
              "LeftElbowPitch": 88.86,
              "LeftElbowYaw": -83.08,
              "RightShoulderPitch": -32.66,
              "RightShoulderRoll": 93.41,
              "RightElbowPitch": 58.44,
              "RightElbowYaw": 45.84,
              "LeftHipPitch": -28.65,
              "LeftHipRoll": 0.0,
              "LeftHipYaw": 0.0,
              "LeftKneePitch": 6.01,
              "CrankUpLeft": -4.58,
              "CrankDownLeft": -4.58,
              "RightHipPitch": -28.65,
              "RightHipRoll": 0.0,
              "RightHipYaw": 0.0,
              "RightKneePitch": 6.01,
              "CrankUpRight": -4.58,
              "CrankDownRight": -4.58
            },
            "name": "step_2",
            "delay_before": 0.0,
            "duration": 2.0
          },
          {
            "joints": {
              "HeadPitch": 0.0,
              "LeftShoulderPitch": -70.47,
              "LeftShoulderRoll": -93.97,
              "LeftElbowPitch": 88.86,
              "LeftElbowYaw": -61.88,
              "RightShoulderPitch": -36.12,
              "RightShoulderRoll": 95.10,
              "RightElbowPitch": 67.04,
              "RightElbowYaw": 50.42,
              "LeftHipPitch": -28.65,
              "LeftHipRoll": 0.0,
              "LeftHipYaw": 0.0,
              "LeftKneePitch": 6.01,
              "CrankUpLeft": -4.58,
              "CrankDownLeft": -4.58,
              "RightHipPitch": -28.65,
              "RightHipRoll": 0.0,
              "RightHipYaw": 0.0,
              "RightKneePitch": 6.01,
              "CrankUpRight": -4.58,
              "CrankDownRight": -4.58
            },
            "name": "step_3",
            "delay_before": 0.0,
            "duration": 2.0
          }
        ]
      }
    }
    )JSON";

    Parser parser;
    const auto action = parser.parse_string_json(json);

    EXPECT_EQ(action.action_name, "waving_flag");
    EXPECT_EQ(action.control_type, ControlType::UPPER_BODY);
    EXPECT_EQ(action.next_mode, "");

    ASSERT_FALSE(action.trajectory.joint_names.empty());
    ASSERT_EQ(action.trajectory.points.size(), 4u);

    expect_point_matches_map(action, 0, base_joint_degrees());
    expect_point_matches_map(action, 1, point_1_degrees());
    expect_point_matches_map(action, 2, point_2_degrees());
    expect_point_matches_map(action, 3, point_3_degrees());

    for (const auto & point : action.trajectory.points) {
        ASSERT_EQ(point.velocities.size(), point.positions.size());
        for (double v : point.velocities) {
            EXPECT_DOUBLE_EQ(v, 0.0);
        }
    }
}