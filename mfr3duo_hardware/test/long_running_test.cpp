#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "mfr3duo_hardware/robot_hardware.hpp"

int main() {
    using mfr3duo_hardware::Camera;
    using mfr3duo_hardware::CameraFrame;
    using mfr3duo_hardware::LaserScan;
    using mfr3duo_hardware::Lidar;
    using mfr3duo_hardware::RobotHardware;
    using mfr3duo_hardware::RobotHardwareOptions;
    using mfr3duo_hardware::RobotState;

    RobotHardwareOptions options;
    options.control_period = std::chrono::milliseconds(2);
    RobotHardware robot;
    if (!robot.initialize(options) || !robot.activate()) return EXIT_FAILURE;

    RobotState before;
    if (!robot.read_state(before)) return EXIT_FAILURE;

    constexpr std::size_t kCyclesPerBatch = 500;
    constexpr std::uint64_t kBatches = 60;
    std::uint64_t previous_sequence = before.sequence;
    std::uint64_t previous_timestamp = before.timestamp_ns;
    for (std::uint64_t batch = 0; batch < kBatches; ++batch) {
        for (std::size_t cycle = 0; cycle < kCyclesPerBatch; ++cycle) {
            if (!robot.update()) {
                std::cerr << "control cycle failed at batch " << batch << '\n';
                return EXIT_FAILURE;
            }
        }
        RobotState state;
        LaserScan lidar;
        CameraFrame camera;
        if (!robot.read_state(state) || !robot.read_state(Lidar::Front, lidar) ||
            !robot.read_state(Camera::FrontColor, camera) || state.sequence <= previous_sequence ||
            state.timestamp_ns <= previous_timestamp || lidar.ranges.empty() ||
            camera.image.data.empty()) {
            std::cerr << "state or sensor stream invalid at batch " << batch << '\n';
            return EXIT_FAILURE;
        }
        previous_sequence = state.sequence;
        previous_timestamp = state.timestamp_ns;
    }
    if (!robot.deactivate() || !robot.shutdown()) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
