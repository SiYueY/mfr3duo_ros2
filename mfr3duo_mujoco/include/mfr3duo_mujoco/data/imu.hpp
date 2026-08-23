#pragma once
#include "mfr3duo_mujoco/data/math.hpp"
namespace mfr3duo_mujoco {

struct ImuState {
    double timestamp{0.0};
    Quaterniond orientation{1., 0., 0., 0.};
    Vector3d angular_velocity{};
    Vector3d linear_acceleration{};
};

}  // namespace mfr3duo_mujoco
