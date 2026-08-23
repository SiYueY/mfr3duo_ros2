#pragma once
#include <cstdint>
namespace mfr3duo_mujoco {

enum class JointMode : std::uint8_t { None = 0, Position, Velocity, Effort, Hybrid };

struct JointCommand {
    JointMode mode{JointMode::None};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double stiffness{0.0};
    double damping{0.0};
};

struct JointState {
    JointMode mode{JointMode::None};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};

}  // namespace mfr3duo_mujoco
