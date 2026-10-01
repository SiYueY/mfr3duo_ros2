// Regression guard for the motion snapshot concurrency contract: the control
// thread publishes snapshots while reader threads (the Ros2SensorAdapter
// pattern) read them, and the reader must never observe a torn snapshot.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "mfr3duo_hardware/robot_hardware.hpp"

namespace {

using mfr3duo_hardware::JointControlMode;
using mfr3duo_hardware::JointState;
using mfr3duo_hardware::RobotCommand;
using mfr3duo_hardware::RobotHardware;
using mfr3duo_hardware::RobotHardwareOptions;
using mfr3duo_hardware::RobotState;

constexpr std::size_t kReaders = 2;
constexpr auto kRunBudget = std::chrono::seconds(15);

bool same_joint(const JointState& left, const JointState& right) {
    return left.position == right.position && left.velocity == right.velocity &&
           left.effort == right.effort;
}

// Every field of a published snapshot is written together, so two observations
// that report the same sequence must be identical. A torn read violates this.
bool same_snapshot(const RobotState& left, const RobotState& right) {
    if (left.sequence != right.sequence || left.timestamp_ns != right.timestamp_ns) return false;
    if (left.spine.position != right.spine.position) return false;
    if (left.spine.velocity != right.spine.velocity) return false;
    const std::array<JointState, 4> left_tmr{
        left.tmr.front_steering, left.tmr.front_drive, left.tmr.rear_steering, left.tmr.rear_drive};
    const std::array<JointState, 4> right_tmr{
        right.tmr.front_steering, right.tmr.front_drive, right.tmr.rear_steering,
        right.tmr.rear_drive};
    for (std::size_t index = 0; index < left_tmr.size(); ++index) {
        if (!same_joint(left_tmr[index], right_tmr[index])) return false;
    }
    for (std::size_t index = 0; index < left.left_arm.joints.size(); ++index) {
        if (!same_joint(left.left_arm.joints[index], right.left_arm.joints[index])) return false;
        if (!same_joint(left.right_arm.joints[index], right.right_arm.joints[index])) return false;
    }
    if (left.left_gripper.width != right.left_gripper.width ||
        left.left_gripper.velocity != right.left_gripper.velocity ||
        left.left_gripper.effort != right.left_gripper.effort ||
        left.left_gripper.stalled != right.left_gripper.stalled) {
        return false;
    }
    return left.right_gripper.width == right.right_gripper.width &&
           left.right_gripper.velocity == right.right_gripper.velocity &&
           left.right_gripper.effort == right.right_gripper.effort &&
           left.right_gripper.stalled == right.right_gripper.stalled;
}

bool snapshot_is_finite(const RobotState& state) {
    const auto finite = [](double value) { return std::isfinite(value) != 0; };
    if (!finite(state.spine.position) || !finite(state.spine.velocity)) return false;
    const std::array<JointState, 4> tmr{
        state.tmr.front_steering, state.tmr.front_drive, state.tmr.rear_steering,
        state.tmr.rear_drive};
    for (const auto& joint : tmr) {
        if (!finite(joint.position) || !finite(joint.velocity) || !finite(joint.effort)) {
            return false;
        }
    }
    for (const auto& joint : state.left_arm.joints) {
        if (!finite(joint.position) || !finite(joint.velocity) || !finite(joint.effort)) {
            return false;
        }
    }
    for (const auto& joint : state.right_arm.joints) {
        if (!finite(joint.position) || !finite(joint.velocity) || !finite(joint.effort)) {
            return false;
        }
    }
    return finite(state.left_gripper.width) && finite(state.right_gripper.width);
}

RobotCommand hold_command(const RobotState& state) {
    RobotCommand command;
    command.left_arm.mode = JointControlMode::Position;
    command.right_arm.mode = JointControlMode::Position;
    for (std::size_t index = 0; index < state.left_arm.joints.size(); ++index) {
        command.left_arm.joints[index].position = state.left_arm.joints[index].position;
        command.right_arm.joints[index].position = state.right_arm.joints[index].position;
    }
    command.spine.position = state.spine.position;
    command.tmr.front_steering_position = state.tmr.front_steering.position;
    command.tmr.rear_steering_position = state.tmr.rear_steering.position;
    command.left_gripper.width = state.left_gripper.width;
    command.right_gripper.width = state.right_gripper.width;
    return command;
}

}  // namespace

int main() {
    RobotHardware robot;
    RobotHardwareOptions options;
    options.control_period = std::chrono::milliseconds(2);
    if (!robot.initialize(options) || !robot.activate()) return EXIT_FAILURE;

    RobotState initial;
    if (!robot.read_state(initial)) return EXIT_FAILURE;
    const auto command = hold_command(initial);

    std::atomic<bool> stop{false};
    std::atomic<bool> failed{false};
    std::array<std::atomic<std::uint64_t>, kReaders> reads{};
    std::array<std::thread, kReaders> readers;
    for (std::size_t index = 0; index < kReaders; ++index) {
        readers[index] = std::thread([&, index] {
            RobotState previous;
            bool have_previous = false;
            while (!stop.load(std::memory_order_relaxed)) {
                RobotState current;
                if (!robot.read_state(current)) {
                    std::cerr << "reader " << index << " lost the snapshot\n";
                    failed.store(true);
                    return;
                }
                if (!snapshot_is_finite(current)) {
                    std::cerr << "reader " << index << " observed a non-finite snapshot\n";
                    failed.store(true);
                    return;
                }
                if (have_previous) {
                    if (current.sequence < previous.sequence) {
                        std::cerr << "reader " << index << " observed a sequence regression\n";
                        failed.store(true);
                        return;
                    }
                    if (current.sequence == previous.sequence &&
                        !same_snapshot(current, previous)) {
                        std::cerr << "reader " << index << " observed a torn snapshot at sequence "
                                  << current.sequence << '\n';
                        failed.store(true);
                        return;
                    }
                }
                previous = current;
                have_previous = true;
                reads[index].fetch_add(1, std::memory_order_relaxed);
            }
        });
    }

    const auto deadline = std::chrono::steady_clock::now() + kRunBudget;
    while (std::chrono::steady_clock::now() < deadline && !failed.load()) {
        if (!robot.write_command(command) || !robot.update()) {
            std::cerr << "control cycle failed\n";
            failed.store(true);
            break;
        }
    }

    stop.store(true);
    for (auto& reader : readers) reader.join();
    if (failed.load()) return EXIT_FAILURE;

    for (std::size_t index = 0; index < kReaders; ++index) {
        if (reads[index].load() == 0) {
            std::cerr << "reader " << index << " never observed a snapshot\n";
            return EXIT_FAILURE;
        }
    }
    std::cout << "RobotHardware concurrency: control thread and " << kReaders
              << " readers stayed coherent\n";
    if (!robot.deactivate() || !robot.shutdown()) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
