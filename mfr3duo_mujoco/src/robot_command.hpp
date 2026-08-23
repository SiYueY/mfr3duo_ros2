#pragma once

#include "mfr3duo_mujoco/data/robot.hpp"
#include "mujoco_simulation/data/robot_command.hpp"

namespace mfr3duo_mujoco {

bool to_runtime_command(const RobotCommand& input, mujoco_simulation::RobotCommand& output);

}  // namespace mfr3duo_mujoco
