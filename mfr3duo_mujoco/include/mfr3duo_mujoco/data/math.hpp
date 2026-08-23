#pragma once

#include <array>

namespace mfr3duo_mujoco {

/// Three-dimensional vector in the public, dependency-free data API.
using Vector3d = std::array<double, 3>;

/// Quaternion in w, x, y, z order.
using Quaterniond = std::array<double, 4>;

}  // namespace mfr3duo_mujoco
