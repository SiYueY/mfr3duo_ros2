#include "mfr3duo_control/tmr_odometry.hpp"
#include <algorithm>
#include <cmath>

namespace mfr3duo_control {
bool TmrOdometry::configure(
    const std::array<ModuleGeometry, 2>& modules, double radius,
    const OdometryLimits& limits) noexcept {
    configured_ = false;
    if (!TmrKinematics::valid_geometry(modules, radius)) return false;
    for (double value :
         {limits.maximum_condition, limits.maximum_timestamp_gap, limits.maximum_encoder_delta,
          limits.steering_interval, limits.residual_ratio})
        if (!std::isfinite(value) || value <= 0) return false;
    const double sx = modules[0].x + modules[1].x;
    const double sy = modules[0].y + modules[1].y;
    const double squared = modules[0].x * modules[0].x + modules[0].y * modules[0].y +
                           modules[1].x * modules[1].x + modules[1].y * modules[1].y;
    // Eigenvalues of A^T A: 2 and the roots of [2, sqrt(sx²+sy²); ..., squared].
    const double discriminant = std::hypot(2 - squared, 2 * std::hypot(sx, sy));
    const double high = (2 + squared + discriminant) / 2;
    const double determinant = 2 * squared - sx * sx - sy * sy;
    const double low = determinant / high;
    const double condition = std::max(2., high) / std::min(2., low);
    if (!std::isfinite(condition) || low <= 1e-9 || condition > limits.maximum_condition)
        return false;
    modules_ = modules;
    radius_ = radius;
    limits_ = limits;
    center_x_ = sx / 2;
    center_y_ = sy / 2;
    denominator_ = squared - (sx * sx + sy * sy) / 2;
    estimate_ = {};
    estimate_.condition = condition;
    baseline_ = false;
    configured_ = true;
    return true;
}
void TmrOdometry::reset_baseline() noexcept {
    baseline_ = false;
    estimate_.vx = estimate_.vy = estimate_.wz = 0;
    estimate_.confidence = 0;
    estimate_.updated = false;
    estimate_.status = OdometryStatus::Initialized;
}
const OdometryEstimate& TmrOdometry::update(
    const std::array<double, 2>& steering, const std::array<double, 2>& wheels,
    std::int64_t stamp) noexcept {
    estimate_.updated = false;
    estimate_.vx = estimate_.vy = estimate_.wz = 0;
    estimate_.confidence = 0;
    estimate_.residual = 0;
    if (!configured_ || stamp < 0 || !std::isfinite(steering[0]) || !std::isfinite(steering[1]) ||
        !std::isfinite(wheels[0]) || !std::isfinite(wheels[1])) {
        baseline_ = false;
        estimate_.status = OdometryStatus::InvalidState;
        return estimate_;
    }
    const double dt = static_cast<double>(stamp - timestamp_) * 1e-9;
    const auto baseline = [&] {
        steering_ = steering;
        wheels_ = wheels;
        timestamp_ = stamp;
        baseline_ = true;
    };
    if (!baseline_) {
        baseline();
        estimate_.status = OdometryStatus::Initialized;
        return estimate_;
    }
    if (dt <= 0 || dt > limits_.maximum_timestamp_gap) {
        baseline();
        estimate_.status = OdometryStatus::ClockReset;
        return estimate_;
    }
    std::array<double, 2> bx{}, by{};
    double turn = 0, steering_interval = 0;
    for (std::size_t i = 0; i < 2; ++i) {
        double delta = wheels[i] - wheels_[i];
        // Accept a genuine +/-pi representation wrap; continuous encoder jumps
        // are rejected rather than aliased into a plausible displacement.
        if (std::abs(delta) > kPi && std::abs(wheels[i]) <= kPi && std::abs(wheels_[i]) <= kPi)
            delta = TmrKinematics::shortest_angle(delta);
        if (!std::isfinite(delta) || std::abs(delta) > limits_.maximum_encoder_delta) {
            baseline();
            estimate_.status = OdometryStatus::EncoderReset;
            return estimate_;
        }
        const double steering_delta = TmrKinematics::shortest_angle(steering[i] - steering_[i]);
        if (!std::isfinite(steering_delta)) {
            baseline();
            estimate_.status = OdometryStatus::InvalidState;
            return estimate_;
        }
        steering_interval = std::max(steering_interval, std::abs(steering_delta));
        const auto& m = modules_[i];
        const double angle =
            m.steering_sign * (steering_[i] + steering_delta / 2 - m.steering_offset);
        const double distance = radius_ * m.drive_sign * delta;
        bx[i] = distance * std::cos(angle);
        by[i] = distance * std::sin(angle);
        turn += (m.x - center_x_) * by[i] - (m.y - center_y_) * bx[i];
    }
    turn /= denominator_;
    const double dx = (bx[0] + bx[1]) / 2 + center_y_ * turn;
    const double dy = (by[0] + by[1]) / 2 - center_x_ * turn;
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(turn)) {
        baseline();
        estimate_.status = OdometryStatus::InvalidState;
        return estimate_;
    }
    double residual = 0;
    for (std::size_t i = 0; i < 2; ++i) {
        const double ex = bx[i] - (dx - turn * modules_[i].y);
        const double ey = by[i] - (dy + turn * modules_[i].x);
        residual += ex * ex + ey * ey;
    }
    estimate_.residual = std::sqrt(residual / 4);
    const double distance = std::max(std::hypot(bx[0], by[0]), std::hypot(bx[1], by[1]));
    estimate_.status = OdometryStatus::Valid;
    estimate_.confidence = 1;
    if (steering_interval > limits_.steering_interval) {
        estimate_.confidence = limits_.steering_interval / steering_interval;
        estimate_.status = OdometryStatus::SteeringTransition;
    }
    if (estimate_.residual > 1e-5 + limits_.residual_ratio * distance) {
        estimate_.confidence = std::min(estimate_.confidence, .1);
        estimate_.status = OdometryStatus::Slip;
    }
    const double a = std::abs(turn) < 1e-6 ? 1 - turn * turn / 6 : std::sin(turn) / turn;
    const double b =
        std::abs(turn) < 1e-6 ? turn / 2 - turn * turn * turn / 24 : (1 - std::cos(turn)) / turn;
    const double x = a * dx - b * dy, y = b * dx + a * dy;
    const double next_x = estimate_.x + std::cos(estimate_.yaw) * x - std::sin(estimate_.yaw) * y;
    const double next_y = estimate_.y + std::sin(estimate_.yaw) * x + std::cos(estimate_.yaw) * y;
    if (!std::isfinite(next_x) || !std::isfinite(next_y) || !std::isfinite(dx / dt) ||
        !std::isfinite(dy / dt) || !std::isfinite(turn / dt)) {
        baseline();
        estimate_.confidence = 0;
        estimate_.status = OdometryStatus::InvalidState;
        return estimate_;
    }
    estimate_.x = next_x;
    estimate_.y = next_y;
    estimate_.yaw = TmrKinematics::shortest_angle(estimate_.yaw + turn);
    estimate_.vx = dx / dt;
    estimate_.vy = dy / dt;
    estimate_.wz = turn / dt;
    estimate_.updated = true;
    baseline();
    return estimate_;
}
}  // namespace mfr3duo_control
