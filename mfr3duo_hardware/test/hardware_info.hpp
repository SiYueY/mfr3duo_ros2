#pragma once

#include <cstddef>
#include <initializer_list>
#include <string>

#include "hardware_interface/hardware_info.hpp"

namespace mfr3duo_hardware_test {

inline hardware_interface::InterfaceInfo interface(const char* name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  return result;
}

inline hardware_interface::ComponentInfo component(
    const std::string& name, std::initializer_list<const char*> commands,
    std::initializer_list<const char*> states) {
  hardware_interface::ComponentInfo result;
  result.name = name;
  for (const char* item : commands) result.command_interfaces.push_back(interface(item));
  for (const char* item : states) result.state_interfaces.push_back(interface(item));
  return result;
}

inline hardware_interface::HardwareInfo make_info(std::size_t steps_per_cycle = 2) {
  hardware_interface::HardwareInfo info;
  info.name = "MFR3DuoMujoco";
  info.type = "system";
  info.hardware_class_type = "mfr3duo_hardware/MujocoSystem";
  info.hardware_parameters["simulation_steps_per_cycle"] = std::to_string(steps_per_cycle);
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

}  // namespace mfr3duo_hardware_test
