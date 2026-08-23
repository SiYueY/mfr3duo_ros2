#pragma once

#include "mfr3duo_mujoco/data/robot.hpp"
#include "mujoco_simulation/data/robot_state.hpp"

namespace mfr3duo_mujoco {

bool from_runtime_state(const mujoco_simulation::RobotState& input, RobotState& output);

}  // namespace mfr3duo_mujoco
