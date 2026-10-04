#include "mfr3duo_control/tmr_odometry.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>

namespace {
thread_local bool counting = false;
thread_local unsigned allocations = 0;
void require(bool value, const char* detail) {
    if (!value) {
        std::cerr << detail << '\n';
        std::exit(1);
    }
}
void near(double value, double expected) {
    require(std::abs(value - expected) < 1e-10, "odometry value");
}
using namespace mfr3duo_control;
const std::array<ModuleGeometry, 2> geometry{{{.3, -.2, 0, 1, 1}, {-.3, .2, 0, 1, 1}}};
void displacement(const std::array<ModuleGeometry, 2>& modules, double dx, double dy, double turn) {
    TmrOdometry odometry;
    require(odometry.configure(modules, .05), "configure");
    std::array<double, 2> angles{}, wheels{};
    for (unsigned i = 0; i < 2; ++i) {
        const double x = dx - turn * modules[i].y;
        const double y = dy + turn * modules[i].x;
        angles[i] = modules[i].steering_offset + modules[i].steering_sign * std::atan2(y, x);
        wheels[i] = modules[i].drive_sign * std::hypot(x, y) / .05;
    }
    odometry.update(angles, {0, 0}, 0);
    const auto& actual = odometry.update(angles, wheels, 100000000);
    require(
        actual.updated && actual.status == OdometryStatus::Valid,
        "consistent displacement rejected");
    near(actual.vx, dx / .1);
    near(actual.vy, dy / .1);
    near(actual.wz, turn / .1);
    if (turn == 0) {
        near(actual.x, dx);
        near(actual.y, dy);
    } else if (dx == 0 && dy == 0) {
        near(actual.x, 0);
        near(actual.y, 0);
    }
    near(actual.yaw, turn);
    near(actual.residual, 0);
}
}  // namespace
void* operator new(std::size_t size) {
    if (counting) ++allocations;
    if (void* value = std::malloc(size ? size : 1)) return value;
    throw std::bad_alloc();
}
void operator delete(void* value) noexcept { std::free(value); }
void operator delete(void* value, std::size_t) noexcept { std::free(value); }
int main() {
    displacement(geometry, .005, 0, 0);
    displacement(geometry, 0, .005, 0);
    displacement(geometry, .003, .004, 0);
    displacement(geometry, 0, 0, .01);
    auto offset = geometry;
    offset[0].x += .4;
    offset[1].x += .4;
    offset[0].y -= .1;
    offset[1].y -= .1;
    offset[0].steering_offset = .25;
    offset[0].steering_sign = -1;
    offset[0].drive_sign = -1;
    offset[1].steering_offset = -.7;
    displacement(offset, 0, 0, .01);
    displacement(offset, .003, .002, .005);

    TmrOdometry fast, slow;
    require(fast.configure(geometry, .05) && slow.configure(geometry, .05), "clock configurations");
    fast.update({0, 0}, {0, 0}, 0);
    slow.update({0, 0}, {0, 0}, 0);
    const auto a = fast.update({0, 0}, {.1, .1}, 20000000);
    const auto b = slow.update({0, 0}, {.1, .1}, 100000000);
    near(a.x, b.x);
    near(a.vx, 5 * b.vx);

    TmrOdometry odometry;
    require(odometry.configure(geometry, .05), "configure diagnostics");
    odometry.update({0, 0}, {kPi - .05, kPi - .05}, 0);
    near(odometry.update({0, 0}, {-kPi + .05, -kPi + .05}, 100000000).x, .005);
    const auto before = odometry.estimate().x;
    require(
        odometry.update({0, 0}, {20, 20}, 200000000).status == OdometryStatus::EncoderReset,
        "continuous encoder reset aliased");
    near(odometry.estimate().x, before);
    require(
        odometry.update({0, 0}, {20.1, 20.1}, 0).status == OdometryStatus::ClockReset,
        "negative ROS delta integrated");
    near(odometry.estimate().x, before);
    require(
        odometry.update({0, 0}, {20.2, 20.2}, 0).status == OdometryStatus::ClockReset,
        "zero ROS delta integrated");
    require(
        odometry.update({0, 0}, {20.3, 20.3}, 1000000000).status == OdometryStatus::ClockReset,
        "ROS clock jump integrated");
    near(odometry.estimate().x, before);
    odometry.reset_baseline();
    odometry.update({0, 0}, {0, 0}, 1010000000);
    near(odometry.estimate().x, before);
    near(odometry.update({0, 0}, {.1, .1}, 1110000000).x, before + .005);
    require(
        odometry.update({NAN, 0}, {.2, .2}, 1120000000).status == OdometryStatus::InvalidState,
        "NaN accepted");
    odometry.update({0, 0}, {.2, .2}, 1130000000);
    require(
        odometry.update({0, 0}, {.3, .1}, 1230000000).status == OdometryStatus::Slip,
        "module disagreement unreported");
    require(
        odometry.estimate().confidence <= .1 && odometry.estimate().residual > 0,
        "slip confidence");
    odometry.reset_baseline();
    odometry.update({0, 0}, {0, 0}, 0);
    require(
        odometry.update({.4, .4}, {.1, .1}, 100000000).status == OdometryStatus::SteeringTransition,
        "large steering interval unreported");
    near(odometry.estimate().confidence, .25);
    auto invalid = geometry;
    invalid[1] = invalid[0];
    require(!odometry.configure(invalid, .05), "rank deficient geometry accepted");
    invalid = geometry;
    invalid[0].x += 1e5;
    invalid[1].x += 1e5;
    require(!odometry.configure(invalid, .05), "ill-conditioned geometry accepted");

    require(odometry.configure(geometry, .05), "allocation configuration");
    odometry.update({0, 0}, {0, 0}, 0);
    counting = true;
    for (int i = 1; i <= 10000; ++i) odometry.update({0, 0}, {.001 * i, .001 * i}, i * 1000000LL);
    counting = false;
    require(allocations == 0, "odometry update allocated");
    near(odometry.estimate().x, .5);
    std::cout << "TMR_ODOMETRY_PASS rank/condition, signed modules, motion, clock/encoder reset, "
                 "residual, allocation\n";
}
