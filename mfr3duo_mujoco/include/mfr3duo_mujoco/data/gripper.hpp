#pragma once
#include "mfr3duo_mujoco/data/joint.hpp"
namespace mfr3duo_mujoco {

struct GripperCommand {
    JointCommand actuator;
};

struct GripperState {
    JointState left_finger;
    JointState right_finger;
};

}  // namespace mfr3duo_mujoco
