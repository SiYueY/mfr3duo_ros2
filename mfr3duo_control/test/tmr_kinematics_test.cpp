#include <cmath>
#include <iostream>
#include <limits>

#include "mfr3duo_control/tmr_kinematics.hpp"

using namespace mfr3duo_control;

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}
bool near(double a, double b) { return std::abs(a - b) < 1e-10; }

int main() {
    std::array<ModuleGeometry, 2> geometry{{{0.3, -0.2, 0, 1, 1}, {-0.3, 0.2, 0, 1, 1}}};
    std::array<ModuleCommand, 2> result{};
    if (!check(
            static_cast<bool>(TmrKinematics::inverse(geometry, .05, .1, 0, 0, {0, 0}, result)),
            "forward rejected") ||
        !check(near(result[0].drive, 2) && near(result[1].drive, 2), "forward speed") ||
        !check(near(result[0].steering, 0), "forward direction"))
        return 1;
    TmrKinematics::inverse(geometry, .05, 0, .1, 0, {0, 0}, result);
    if (!check(near(result[0].steering, kPi / 2) && near(result[1].drive, 2), "pi/2 tie") ||
        !check(near(TmrKinematics::shortest_angle(-kPi), kPi), "negative pi tie") ||
        !check(near(TmrKinematics::shortest_angle(kPi), kPi), "positive pi tie"))
        return 1;
    TmrKinematics::inverse(geometry, .05, -.1, 0, 0, {0, 0}, result);
    if (!check(near(result[0].steering, 0) && near(result[0].drive, -2), "wheel reversal"))
        return 1;
    TmrKinematics::inverse(geometry, .05, .1, 0, 0, {2 * kPi + .1, -2 * kPi - .1}, result);
    if (!check(
            near(result[0].steering, 2 * kPi) && near(result[1].steering, -2 * kPi),
            "continuous nearest angle"))
        return 1;
    TmrKinematics::inverse(geometry, .05, 0, 0, .2, {0, 0}, result);
    for (std::size_t i = 0; i < 2; ++i) {
        const double vx = result[i].drive * .05 * std::cos(result[i].steering);
        const double vy = result[i].drive * .05 * std::sin(result[i].steering);
        if (!check(
                near(vx, -.2 * geometry[i].y) && near(vy, .2 * geometry[i].x),
                "signed rotation geometry"))
            return 1;
    }
    geometry[0].steering_offset = .25;
    geometry[0].steering_sign = -1;
    geometry[0].drive_sign = -1;
    TmrKinematics::inverse(geometry, .05, 0, .1, 0, {.25, 0}, result);
    if (!check(
            near(result[0].steering, .25 - kPi / 2) && near(result[0].drive, -2),
            "mount signs/offset"))
        return 1;
    TmrKinematics::inverse(geometry, .05, 0, 0, 0, {2.5, -3}, result);
    if (!check(
            near(result[0].steering, 2.5) && near(result[1].steering, -3) && result[0].drive == 0,
            "zero hold"))
        return 1;
    const auto saved = result;
    if (!check(
            !TmrKinematics::inverse(geometry, 0, .1, 0, 0, {0, 0}, result),
            "zero radius accepted") ||
        !check(
            !TmrKinematics::inverse(
                geometry, .05, std::numeric_limits<double>::quiet_NaN(), 0, 0, {0, 0}, result),
            "NaN accepted") ||
        !check(
            !TmrKinematics::inverse(
                geometry, .05, .1, 0, 0, {0, std::numeric_limits<double>::infinity()}, result),
            "infinite steering accepted") ||
        !check(result[0].steering == saved[0].steering, "failed IK modified output"))
        return 1;
    geometry[1] = geometry[0];
    if (!check(!TmrKinematics::valid_geometry(geometry, .05), "degenerate geometry accepted") ||
        !check(
            near(TmrKinematics::drive_scale(.1, .1, .6), 1) &&
                near(TmrKinematics::drive_scale(.6, .1, .6), 0) &&
                near(TmrKinematics::drive_scale(.35, .1, .6), .5),
            "alignment limiting"))
        return 1;
    std::cout << "TMR IK: forward/lateral/rotation/signs/ties/reversal/zero/invalid passed\n";
}
