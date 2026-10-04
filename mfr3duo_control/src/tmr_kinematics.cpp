#include "mfr3duo_control/tmr_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace mfr3duo_control {

double TmrKinematics::shortest_angle(double difference) noexcept {
    double angle = std::remainder(difference, 2.0 * kPi);
    // Both representations of the pi tie use positive rotation.
    if (angle == -kPi) angle = kPi;
    return angle;
}

bool TmrKinematics::valid_geometry(
    const std::array<ModuleGeometry, 2>& modules, double radius) noexcept {
    if (!std::isfinite(radius) || radius <= 0.0) return false;
    for (const auto& module : modules) {
        if (!std::isfinite(module.x) || !std::isfinite(module.y) ||
            !std::isfinite(module.steering_offset) ||
            (module.steering_sign != 1.0 && module.steering_sign != -1.0) ||
            (module.drive_sign != 1.0 && module.drive_sign != -1.0))
            return false;
    }
    const double separation = std::hypot(modules[0].x - modules[1].x, modules[0].y - modules[1].y);
    return std::isfinite(separation) && separation > 1e-9;
}

KinematicsResult TmrKinematics::inverse(
    const std::array<ModuleGeometry, 2>& modules, double radius, double vx, double vy, double wz,
    const std::array<double, 2>& steering, std::array<ModuleCommand, 2>& output) noexcept {
    if (!valid_geometry(modules, radius) || !std::isfinite(vx) || !std::isfinite(vy) ||
        !std::isfinite(wz) || !std::isfinite(steering[0]) || !std::isfinite(steering[1])) {
        return {KinematicsError::InvalidArgument};
    }
    std::array<ModuleCommand, 2> result{};
    for (std::size_t i = 0; i < modules.size(); ++i) {
        const auto& m = modules[i];
        const double x = vx - wz * m.y;
        const double y = vy + wz * m.x;
        if (!std::isfinite(x) || !std::isfinite(y)) return {KinematicsError::InvalidArgument};
        result[i].steering = steering[i];
        const double speed = std::hypot(x, y);
        if (speed < 1e-9) continue;
        const double current = m.steering_sign * (steering[i] - m.steering_offset);
        double delta = shortest_angle(std::atan2(y, x) - current);
        double drive = speed / radius;
        // Exactly pi/2 retains the unreversed solution, deterministically.
        if (std::abs(delta) > kPi / 2.0) {
            delta = shortest_angle(delta + kPi);
            drive = -drive;
        }
        result[i] = {steering[i] + m.steering_sign * delta, m.drive_sign * drive};
        if (!std::isfinite(result[i].steering) || !std::isfinite(result[i].drive)) {
            return {KinematicsError::InvalidArgument};
        }
    }
    output = result;
    return {};
}

double TmrKinematics::drive_scale(double error, double slow, double stop) noexcept {
    error = std::abs(error);
    if (!std::isfinite(error) || !std::isfinite(slow) || !std::isfinite(stop) || slow < 0.0 ||
        stop <= slow || stop > kPi / 2.0)
        return 0.0;
    if (error >= stop) return 0.0;
    if (error <= slow) return 1.0;
    return (stop - error) / (stop - slow);
}

}  // namespace mfr3duo_control
