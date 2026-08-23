#pragma once
#include "mfr3duo_mujoco/data/joint.hpp"
#include "mfr3duo_mujoco/data/math.hpp"
namespace mfr3duo_mujoco {

struct BaseState {
    Vector3d position{};
    Quaterniond orientation{1., 0., 0., 0.};
    Vector3d linear_velocity{};
    Vector3d angular_velocity{};
};

struct TmrCommand {
    double front_steering_position{0.0};
    double front_drive_velocity{0.0};
    double rear_steering_position{0.0};
    double rear_drive_velocity{0.0};
};

struct TmrState {
    BaseState base;
    JointState front_steering;
    JointState front_drive;
    JointState rear_steering;
    JointState rear_drive;
    JointState rocker;
    JointState front_caster_steering;
    JointState front_caster_rolling;
    JointState rear_caster_steering;
    JointState rear_caster_rolling;
};

}  // namespace mfr3duo_mujoco
