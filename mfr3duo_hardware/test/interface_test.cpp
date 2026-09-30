#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "mfr3duo_hardware/mujoco_system.hpp"

namespace {

hardware_interface::InterfaceInfo interface(const char* name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  return result;
}

hardware_interface::ComponentInfo component(
    const std::string& name, std::initializer_list<const char*> commands,
    std::initializer_list<const char*> states) {
  hardware_interface::ComponentInfo result;
  result.name = name;
  for (const char* item : commands) result.command_interfaces.push_back(interface(item));
  for (const char* item : states) result.state_interfaces.push_back(interface(item));
  return result;
}

hardware_interface::HardwareInfo make_info() {
  hardware_interface::HardwareInfo info;
  info.name = "MFR3DuoMujoco";
  info.type = "system";
  info.hardware_class_type = "mfr3duo_hardware/MujocoSystem";
  info.hardware_parameters["simulation_steps_per_cycle"] = "2";
  for (const char* side : {"left", "right"})
    for (int number = 1; number <= 7; ++number)
      info.joints.push_back(component(
          std::string(side) + "_fr3v2_1_joint" + std::to_string(number),
          {"position", "velocity", "effort"}, {"position", "velocity", "effort"}));
  info.joints.push_back(component("franka_spine_vertical_joint", {"position"},
                                  {"position", "velocity"}));
  for (int number = 0; number < 4; ++number)
    info.joints.push_back(component("tmrv0_2_joint_" + std::to_string(number),
                                    {number % 2 == 0 ? "position" : "velocity"},
                                    {"position", "velocity"}));
  for (const char* side : {"left", "right"})
    info.gpios.push_back(component(std::string(side) + "_gripper",
                                   {"width", "velocity", "effort"},
                                   {"width", "velocity", "effort", "stalled"}));
  info.sensors.push_back(component(
      "imu", {}, {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
                   "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
                   "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}));
  return info;
}

bool check(bool condition, const char* message) {
  if (!condition) std::cerr << message << '\n';
  return condition;
}

}  // namespace

int main() {
  using hardware_interface::CallbackReturn;
  using hardware_interface::return_type;
  mfr3duo_hardware::MujocoSystem system;
  auto info = make_info();
  auto invalid = info;
  invalid.hardware_parameters["simulation_steps_per_cycle"] = "0";
  if (!check(system.on_init(invalid) == CallbackReturn::ERROR, "invalid period accepted") ||
      !check(system.on_init(info) == CallbackReturn::SUCCESS, "initialization failed")) return 1;

  mfr3duo_hardware::MujocoSystem reordered;
  auto shuffled = info;
  std::reverse(shuffled.joints.begin(), shuffled.joints.end());
  std::reverse(shuffled.gpios.begin(), shuffled.gpios.end());
  if (!check(reordered.on_init(shuffled) == CallbackReturn::SUCCESS,
             "interface validation depends on URDF declaration order")) return 1;

  const auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  if (!check(states.size() == 70, "wrong state interface count") ||
      !check(commands.size() == 53, "wrong command interface count")) return 1;
  const auto has = [](const auto& items, const std::string& name) {
    return std::any_of(items.begin(), items.end(), [&](const auto& item) {
      return item.get_name() == name;
    });
  };
  if (!check(has(commands, "left_fr3v2_1_joint1/effort"), "left effort missing") ||
      !check(has(commands, "right_fr3v2_1_joint7/velocity"), "right velocity missing") ||
      !check(has(commands, "tmrv0_2_joint_3/velocity"), "TMR drive missing") ||
      !check(has(states, "imu/orientation.w"), "IMU missing") ||
      !check(has(states, "right_gripper/stalled"), "gripper stalled missing")) return 1;

  std::vector<std::string> partial{"left_fr3v2_1_joint1/velocity"};
  if (!check(system.prepare_command_mode_switch(partial, {}) == return_type::ERROR,
             "partial arm mode accepted")) return 1;
  std::vector<std::string> complete;
  for (int joint = 1; joint <= 7; ++joint)
    complete.push_back("left_fr3v2_1_joint" + std::to_string(joint) + "/velocity");
  for (auto& item : commands)
    if (item.get_name() == "right_fr3v2_1_joint1/position") item.set_value(0.123);
  auto mixed = complete;
  mixed.back() = "left_fr3v2_1_joint7/effort";
  if (!check(system.prepare_command_mode_switch(mixed, {}) == return_type::ERROR,
             "mixed arm mode accepted")) return 1;
  if (!check(system.prepare_command_mode_switch(complete, {}) == return_type::OK,
             "complete arm mode rejected") ||
      !check(system.perform_command_mode_switch(complete, {}) == return_type::OK,
             "arm mode switch failed")) return 1;
  if (!check(std::any_of(commands.begin(), commands.end(), [](const auto& item) {
               return item.get_name() == "right_fr3v2_1_joint1/position" &&
                      item.get_value() == 0.123;
             }), "switching left arm reset right arm command")) return 1;
  if (!check(system.prepare_command_mode_switch({}, complete) == return_type::OK,
             "complete arm stop rejected") ||
      !check(system.prepare_command_mode_switch({},
          {"left_fr3v2_1_joint1/position"}) == return_type::ERROR,
          "wrong arm mode stop accepted")) return 1;

  return 0;
}
