#pragma once
#include "mfr3duo_mujoco/data/joint.hpp"
namespace mfr3duo_mujoco {

struct SpineState {
    JointState joint;
};

struct SpineCommand {
    JointCommand joint;
};

}  // namespace mfr3duo_mujoco
