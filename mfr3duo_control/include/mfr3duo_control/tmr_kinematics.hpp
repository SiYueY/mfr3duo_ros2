#pragma once

#include <array>

namespace mfr3duo_control {

constexpr double kPi = 3.14159265358979323846;

struct ModuleGeometry {
    double x{0.0};
    double y{0.0};
    double steering_offset{0.0};
    double steering_sign{1.0};
    double drive_sign{1.0};
};

struct ModuleCommand {
    double steering{0.0};
    double drive{0.0};
};

enum class KinematicsError { Success, InvalidArgument };

struct KinematicsResult {
    KinematicsError code{KinematicsError::Success};
    explicit operator bool() const noexcept { return code == KinematicsError::Success; }
};

/** Pure C++17 inverse kinematics. Angles are continuous hardware joint positions. */
class TmrKinematics {
public:
    static bool valid_geometry(
        const std::array<ModuleGeometry, 2>& modules, double radius) noexcept;
    static KinematicsResult inverse(
        const std::array<ModuleGeometry, 2>& modules, double radius, double vx, double vy,
        double wz, const std::array<double, 2>& steering,
        std::array<ModuleCommand, 2>& output) noexcept;
    static double shortest_angle(double difference) noexcept;
    static double drive_scale(double error, double slow, double stop) noexcept;
};

}  // namespace mfr3duo_control
