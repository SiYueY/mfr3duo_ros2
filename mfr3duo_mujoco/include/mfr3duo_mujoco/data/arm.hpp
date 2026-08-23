#pragma once
#include "mfr3duo_mujoco/data/joint.hpp"
#include <array>
#include <cstddef>
namespace mfr3duo_mujoco {

struct ArmCommand {
    std::array<JointCommand, 7> joints{};
};

struct ArmState {
    std::array<JointState, 7> joints{};
};

}  // namespace mfr3duo_mujoco
