#pragma once

#include "mfr3duo_mujoco/data/arm.hpp"
#include "mfr3duo_mujoco/data/gripper.hpp"
#include "mfr3duo_mujoco/data/imu.hpp"
#include "mfr3duo_mujoco/data/spine.hpp"
#include "mfr3duo_mujoco/data/tmr.hpp"

#include <cstdint>

namespace mfr3duo_mujoco {

/// Complete semantic state of one Mobile FR3 Duo robot.
struct RobotState {
    double time{0.0};
    std::uint64_t step_count{0};
    TmrState tmr;
    SpineState spine;
    ArmState left_arm;
    ArmState right_arm;
    GripperState left_gripper;
    GripperState right_gripper;
    ImuState imu;
};

/// Complete command snapshot for the robot's active joints.
struct RobotCommand {
    TmrCommand tmr;
    SpineCommand spine;
    ArmCommand left_arm;
    ArmCommand right_arm;
    GripperCommand left_gripper;
    GripperCommand right_gripper;
};

}  // namespace mfr3duo_mujoco
