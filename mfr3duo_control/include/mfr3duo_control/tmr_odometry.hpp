#pragma once

#include <array>
#include <cstdint>
#include "mfr3duo_control/tmr_kinematics.hpp"

namespace mfr3duo_control {
enum class OdometryStatus {
    Initialized,
    Valid,
    ClockReset,
    EncoderReset,
    InvalidState,
    SteeringTransition,
    Slip
};
struct OdometryLimits {
    double maximum_condition{1e6};
    double maximum_timestamp_gap{.5};
    double maximum_encoder_delta{.25};
    double steering_interval{.1};
    double residual_ratio{.2};
};
struct OdometryEstimate {
    double x{0}, y{0}, yaw{0};
    double vx{0}, vy{0}, wz{0};
    double confidence{0}, residual{0}, condition{0};
    OdometryStatus status{OdometryStatus::Initialized};
    bool updated{false};
};
/** Encoder displacement estimation. No ROS, heap allocation or command integration. */
class TmrOdometry {
public:
    bool configure(
        const std::array<ModuleGeometry, 2>& modules, double radius,
        const OdometryLimits& limits = {}) noexcept;
    void reset_baseline() noexcept;
    const OdometryEstimate& update(
        const std::array<double, 2>& steering, const std::array<double, 2>& wheels,
        std::int64_t ros_timestamp_ns) noexcept;
    const OdometryEstimate& estimate() const noexcept { return estimate_; }

private:
    std::array<ModuleGeometry, 2> modules_{};
    std::array<double, 2> steering_{}, wheels_{};
    OdometryLimits limits_{};
    OdometryEstimate estimate_{};
    double radius_{0}, center_x_{0}, center_y_{0}, denominator_{0};
    std::int64_t timestamp_{0};
    bool configured_{false}, baseline_{false};
};
}  // namespace mfr3duo_control
