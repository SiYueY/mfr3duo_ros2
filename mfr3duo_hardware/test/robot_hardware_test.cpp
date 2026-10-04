// Standalone RobotHardware tests. These never start ROS.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "mfr3duo_hardware/robot_hardware.hpp"

namespace {

using mfr3duo_hardware::Camera;
using mfr3duo_hardware::CameraFrame;
using mfr3duo_hardware::ImuState;
using mfr3duo_hardware::JointControlMode;
using mfr3duo_hardware::LaserScan;
using mfr3duo_hardware::Lidar;
using mfr3duo_hardware::RobotCommand;
using mfr3duo_hardware::RobotHardware;
using mfr3duo_hardware::RobotHardwareOptions;
using mfr3duo_hardware::RobotState;

constexpr auto kControlPeriod = std::chrono::milliseconds(2);
constexpr std::size_t kCameraCount = 14;
constexpr std::uint32_t kCameraWidth = 320;
constexpr std::uint32_t kCameraHeight = 180;

bool check(bool condition, const std::string& message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

RobotHardwareOptions make_options() {
    RobotHardwareOptions options;
    options.control_period = kControlPeriod;
    return options;
}

bool advance(RobotHardware& robot, std::size_t cycles) {
    for (std::size_t index = 0; index < cycles; ++index) {
        if (!robot.update()) return false;
    }
    return true;
}

bool finite(double value) { return std::isfinite(value) != 0; }

bool motion_state_is_finite(const RobotState& state) {
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
    if (!finite(state.spine.position) || !finite(state.spine.velocity)) return false;
    const std::array<mfr3duo_hardware::JointState, 4> tmr{
        state.tmr.front_steering, state.tmr.front_drive, state.tmr.rear_steering,
        state.tmr.rear_drive};
    for (const auto& joint : tmr) {
        if (!finite(joint.position) || !finite(joint.velocity) || !finite(joint.effort)) {
            return false;
        }
    }
    const std::array<mfr3duo_hardware::GripperState, 2> grippers{
        state.left_gripper, state.right_gripper};
    for (const auto& gripper : grippers) {
        if (!finite(gripper.width) || !finite(gripper.velocity) || !finite(gripper.effort)) {
            return false;
        }
    }
    return true;
}

double max_joint_velocity(const RobotState& state) {
    double worst = 0.0;
    const auto consider = [&worst](double velocity) {
        worst = std::max(worst, std::fabs(velocity));
    };
    for (const auto& joint : state.left_arm.joints) consider(joint.velocity);
    for (const auto& joint : state.right_arm.joints) consider(joint.velocity);
    consider(state.spine.velocity);
    consider(state.tmr.front_steering.velocity);
    consider(state.tmr.front_drive.velocity);
    consider(state.tmr.rear_steering.velocity);
    consider(state.tmr.rear_drive.velocity);
    consider(state.left_gripper.velocity);
    consider(state.right_gripper.velocity);
    return worst;
}

bool check_uninitialized_rejections(RobotHardware& robot) {
    RobotState state;
    ImuState imu;
    LaserScan scan;
    CameraFrame frame;
    if (!check(!robot.activate(), "activate before initialize was accepted")) return false;
    if (!check(!robot.deactivate(), "deactivate before initialize was accepted")) return false;
    if (!check(!robot.update(), "update before initialize was accepted")) return false;
    if (!check(!robot.write_command(RobotCommand{}), "write before initialize was accepted")) {
        return false;
    }
    if (!check(!robot.read_state(state), "read before initialize was accepted")) return false;
    if (!check(!robot.read_state(imu), "imu read before initialize was accepted")) return false;
    if (!check(
            !robot.read_state(Lidar::Front, scan), "lidar read before initialize was accepted")) {
        return false;
    }
    if (!check(
            !robot.read_state(Camera::FrontColor, frame),
            "camera read before initialize was accepted")) {
        return false;
    }
    return check(robot.shutdown(), "shutdown before initialize failed");
}

bool check_invalid_control_periods() {
    RobotHardware robot;
    RobotHardwareOptions options;
    options.control_period = std::chrono::nanoseconds(0);
    if (!check(!robot.initialize(options), "zero control period was accepted")) return false;
    options.control_period = std::chrono::nanoseconds(-2'000'000);
    if (!check(!robot.initialize(options), "negative control period was accepted")) return false;
    options.control_period = std::chrono::nanoseconds(1'500'000);
    if (!check(!robot.initialize(options), "non-multiple control period was accepted")) {
        return false;
    }
    return check(robot.shutdown(), "shutdown after rejected initialize failed");
}

bool check_safe_activation(RobotHardware& robot, const RobotState& initial) {
    if (!check(robot.activate(), "activate failed")) return false;
    RobotState state;
    if (!check(robot.read_state(state), "read after activate failed")) return false;
    mfr3duo_hardware::BasePoseState base_pose;
    if (!check(robot.read_state(base_pose), "same-instance base pose unavailable")) return false;
    if (!check(base_pose.timestamp_ns <= state.timestamp_ns &&
                   state.timestamp_ns - base_pose.timestamp_ns <= 1'000'001 &&
                   finite(base_pose.position.x) && finite(base_pose.position.y) &&
                   finite(base_pose.orientation.w),
               "base pose sample time or values invalid")) return false;
    if (!check(motion_state_is_finite(state), "state after activate is not finite")) {
        return false;
    }
    // No update() runs between the two activations, so re-activation must hold
    // exactly the same pose instead of snapping somewhere else.
    for (std::size_t index = 0; index < state.left_arm.joints.size(); ++index) {
        const auto& joint = state.left_arm.joints[index];
        if (!check(
                joint.position == initial.left_arm.joints[index].position,
                "activation moved the left arm")) {
            return false;
        }
        const auto& right = state.right_arm.joints[index];
        if (!check(
                right.position == initial.right_arm.joints[index].position,
                "activation moved the right arm")) {
            return false;
        }
    }
    if (!check(state.spine.position == initial.spine.position, "activation moved the spine")) {
        return false;
    }
    if (!check(
            state.tmr.front_steering.position == initial.tmr.front_steering.position &&
                state.tmr.rear_steering.position == initial.tmr.rear_steering.position,
            "activation moved the TMR steering")) {
        return false;
    }
    if (!check(
            state.left_gripper.width == initial.left_gripper.width &&
                state.right_gripper.width == initial.right_gripper.width,
            "activation moved the grippers")) {
        return false;
    }
    // initialize() probes the backend physics period with a single physics step,
    // so the robot is already about 1 ms into its motion. Measured worst case is
    // 0.02 rad/s; anything approaching a control step's worth of motion means the
    // safe command did not hold.
    return check(max_joint_velocity(state) < 0.05, "activation left a large joint velocity");
}

bool check_motion_snapshot(RobotHardware& robot) {
    RobotState first;
    if (!check(robot.read_state(first), "snapshot read failed")) return false;
    if (!check(advance(robot, 25), "control cycles failed")) return false;
    RobotState second;
    if (!check(robot.read_state(second), "second snapshot read failed")) return false;
    if (!check(second.sequence > first.sequence, "RobotState sequence did not advance")) {
        return false;
    }
    if (!check(
            second.timestamp_ns > first.timestamp_ns, "RobotState timestamp_ns did not advance")) {
        return false;
    }
    mfr3duo_hardware::PassiveJointStates passive;
    if (!check(robot.read_state(passive), "passive sensor read failed")) return false;
    if (!check(
            passive.timestamp_ns == second.timestamp_ns, "passive/whole motion timestamp mismatch"))
        return false;
    for (const auto& joint : passive.joints)
        if (!check(
                std::isfinite(joint.position) && std::isfinite(joint.velocity),
                "passive sensor not finite"))
            return false;
    return check(motion_state_is_finite(second), "RobotState is not finite");
}

bool check_arm_control(RobotHardware& robot) {
    RobotState state;
    if (!check(robot.read_state(state), "arm baseline read failed")) return false;
    const double baseline = state.left_arm.joints[0].position;

    RobotCommand command;
    for (std::size_t index = 0; index < state.left_arm.joints.size(); ++index) {
        command.left_arm.joints[index].position = state.left_arm.joints[index].position;
        command.right_arm.joints[index].position = state.right_arm.joints[index].position;
    }
    command.right_arm.mode = JointControlMode::Position;
    command.left_arm.mode = JointControlMode::Position;
    command.left_arm.joints[0].position = baseline + 0.25;
    command.spine.position = state.spine.position;
    command.tmr.front_steering_position = state.tmr.front_steering.position;
    command.tmr.rear_steering_position = state.tmr.rear_steering.position;
    command.left_gripper.width = state.left_gripper.width;
    command.right_gripper.width = state.right_gripper.width;
    if (!check(robot.write_command(command), "position command rejected")) return false;
    if (!check(advance(robot, 250), "position control cycles failed")) return false;
    if (!check(robot.read_state(state), "arm state read failed")) return false;
    if (!check(
            state.left_arm.joints[0].position > baseline + 0.05,
            "left arm did not follow the position command")) {
        return false;
    }

    // Both arms can hold independent control modes.
    command.left_arm.mode = JointControlMode::Velocity;
    command.right_arm.mode = JointControlMode::Effort;
    for (std::size_t index = 0; index < state.left_arm.joints.size(); ++index) {
        command.left_arm.joints[index].velocity = 0.0;
        command.right_arm.joints[index].effort = 0.0;
    }
    if (!check(robot.write_command(command), "independent arm modes rejected")) return false;
    if (!check(advance(robot, 25), "independent arm mode cycles failed")) return false;

    command.left_arm.mode = JointControlMode::Position;
    command.right_arm.mode = JointControlMode::Position;
    if (!check(robot.write_command(command), "restored position command rejected")) return false;
    return check(advance(robot, 25), "restore cycles failed");
}

bool check_spine_and_tmr(RobotHardware& robot) {
    RobotState state;
    if (!check(robot.read_state(state), "spine baseline read failed")) return false;
    RobotCommand command;
    for (std::size_t index = 0; index < state.left_arm.joints.size(); ++index) {
        command.left_arm.joints[index].position = state.left_arm.joints[index].position;
        command.right_arm.joints[index].position = state.right_arm.joints[index].position;
    }
    if (!check(robot.write_command(command), "hold command rejected")) return false;

    const double spine_baseline = state.spine.position;
    command.spine.position = spine_baseline + 0.03;
    if (!check(robot.write_command(command), "spine command rejected")) return false;
    if (!check(advance(robot, 250), "spine cycles failed")) return false;
    if (!check(robot.read_state(state), "spine state read failed")) return false;
    if (!check(state.spine.position > spine_baseline + 0.01, "spine did not move")) return false;

    const double steering_baseline = state.tmr.front_steering.position;
    command.spine.position = state.spine.position;
    command.tmr.front_steering_position = steering_baseline + 0.2;
    command.tmr.rear_steering_position = state.tmr.rear_steering.position;
    if (!check(robot.write_command(command), "steering command rejected")) return false;
    if (!check(advance(robot, 250), "steering cycles failed")) return false;
    if (!check(robot.read_state(state), "steering state read failed")) return false;
    if (!check(
            state.tmr.front_steering.position > steering_baseline + 0.05,
            "TMR steering did not move")) {
        return false;
    }

    const double drive_baseline = state.tmr.front_drive.velocity;
    command.tmr.front_steering_position = state.tmr.front_steering.position;
    command.tmr.rear_steering_position = state.tmr.rear_steering.position;
    command.tmr.front_drive_velocity = 1.5;
    command.tmr.rear_drive_velocity = 1.5;
    if (!check(robot.write_command(command), "drive command rejected")) return false;
    if (!check(advance(robot, 100), "drive cycles failed")) return false;
    if (!check(robot.read_state(state), "drive state read failed")) return false;
    if (!check(state.tmr.front_drive.velocity > drive_baseline + 0.1, "TMR drive did not move")) {
        return false;
    }
    command.tmr.front_drive_velocity = 0.0;
    command.tmr.rear_drive_velocity = 0.0;
    return check(robot.write_command(command), "drive stop rejected");
}

bool check_grippers(RobotHardware& robot) {
    RobotState state;
    if (!check(robot.read_state(state), "gripper baseline read failed")) return false;
    RobotCommand command;
    for (std::size_t index = 0; index < state.left_arm.joints.size(); ++index) {
        command.left_arm.joints[index].position = state.left_arm.joints[index].position;
        command.right_arm.joints[index].position = state.right_arm.joints[index].position;
    }
    command.spine.position = state.spine.position;
    command.tmr.front_steering_position = state.tmr.front_steering.position;
    command.tmr.rear_steering_position = state.tmr.rear_steering.position;

    const double left_baseline = state.left_gripper.width;
    // velocity is a speed magnitude and effort caps the finger force, so both
    // must be positive for the gripper to move at all.
    command.left_gripper.width = left_baseline > 0.04 ? left_baseline - 0.02 : left_baseline + 0.02;
    command.left_gripper.velocity = 0.05;
    command.left_gripper.effort = 20.0;
    command.right_gripper.width = state.right_gripper.width;
    if (!check(robot.write_command(command), "left gripper command rejected")) return false;
    if (!check(advance(robot, 250), "left gripper cycles failed")) return false;
    if (!check(robot.read_state(state), "gripper state read failed")) return false;
    if (!check(
            std::fabs(state.left_gripper.width - left_baseline) > 0.005,
            "left gripper did not move")) {
        return false;
    }

    command.left_gripper.width = state.left_gripper.width;
    command.left_gripper.velocity = 0.0;
    command.left_gripper.effort = 0.0;
    command.right_gripper.width = state.right_gripper.width;
    if (!check(robot.write_command(command), "gripper hold rejected")) return false;
    return check(advance(robot, 25), "gripper hold cycles failed");
}

bool check_atomic_command_rejection(RobotHardware& robot) {
    RobotState before;
    if (!check(robot.read_state(before), "atomic baseline read failed")) return false;

    RobotCommand command;
    for (std::size_t index = 0; index < before.left_arm.joints.size(); ++index) {
        command.left_arm.joints[index].position = before.left_arm.joints[index].position;
        command.right_arm.joints[index].position = before.right_arm.joints[index].position;
    }
    command.spine.position = before.spine.position;
    command.tmr.front_steering_position = before.tmr.front_steering.position;
    command.tmr.rear_steering_position = before.tmr.rear_steering.position;
    command.left_gripper.width = before.left_gripper.width;
    command.right_gripper.width = before.right_gripper.width;

    // The left arm member is valid, the right arm member is not. Nothing may be
    // committed, so the valid left arm member must not move either.
    const double left_target = before.left_arm.joints[0].position + 0.5;
    command.left_arm.joints[0].position = left_target;
    command.right_arm.joints[0].position = std::numeric_limits<double>::quiet_NaN();
    if (!check(!robot.write_command(command), "command with a NaN member was accepted")) {
        return false;
    }
    if (!check(advance(robot, 250), "cycles after rejected command failed")) return false;
    RobotState after;
    if (!check(robot.read_state(after), "atomic state read failed")) return false;
    if (!check(motion_state_is_finite(after), "rejected command corrupted the state")) {
        return false;
    }
    if (!check(
            std::fabs(after.left_arm.joints[0].position - before.left_arm.joints[0].position) <
                0.05,
            "part of a rejected command was committed")) {
        return false;
    }

    command.right_arm.joints[0].position = before.right_arm.joints[0].position;
    command.left_arm.joints[0].position = before.left_arm.joints[0].position;
    if (!check(robot.write_command(command), "recovery command rejected")) return false;
    return check(advance(robot, 25), "recovery cycles failed");
}

bool check_imu(RobotHardware& robot) {
    ImuState imu;
    if (!check(robot.read_state(imu), "imu read failed")) return false;
    if (!check(imu.sequence > 0, "imu sequence is empty")) return false;
    const double norm = std::sqrt(
        imu.orientation.x * imu.orientation.x + imu.orientation.y * imu.orientation.y +
        imu.orientation.z * imu.orientation.z + imu.orientation.w * imu.orientation.w);
    if (!check(std::fabs(norm - 1.0) < 1.0e-3, "imu orientation is not normalised")) return false;
    return check(
        finite(imu.angular_velocity.x) && finite(imu.angular_velocity.y) &&
            finite(imu.angular_velocity.z) && finite(imu.linear_acceleration.x) &&
            finite(imu.linear_acceleration.y) && finite(imu.linear_acceleration.z),
        "imu values are not finite");
}

bool check_lidar(RobotHardware& robot, Lidar id) {
    LaserScan scan;
    if (!check(robot.read_state(id, scan), "lidar read failed")) return false;
    if (!check(!scan.ranges.empty(), "lidar ranges are empty")) return false;
    if (!check(scan.angle_max > scan.angle_min, "lidar angle range is invalid")) return false;
    if (!check(scan.range_max > scan.range_min, "lidar range limits are invalid")) return false;
    return check(!scan.frame_id.empty(), "lidar frame_id is empty");
}

bool check_invalid_device_ids(RobotHardware& robot) {
    LaserScan scan;
    CameraFrame frame;
    if (!check(
            !robot.read_state(static_cast<Lidar>(2), scan),
            "lidar id past the last device was accepted")) {
        return false;
    }
    if (!check(
            !robot.read_state(static_cast<Lidar>(255), scan),
            "out-of-range lidar id was accepted")) {
        return false;
    }
    if (!check(
            !robot.read_state(static_cast<Camera>(14), frame),
            "camera id past the last device was accepted")) {
        return false;
    }
    if (!check(
            !robot.read_state(static_cast<Camera>(255), frame),
            "out-of-range camera id was accepted")) {
        return false;
    }
    // The real boundary device must still be reachable.
    return check(robot.read_state(Camera::HeadZedRight, frame), "last camera id was rejected");
}

bool check_camera(const CameraFrame& frame) {
    if (!check(frame.sequence > 0, "camera sequence is empty")) return false;
    if (!check(!frame.frame_id.empty(), "camera frame_id is empty")) return false;
    if (!check(!frame.optical_frame_id.empty(), "camera optical_frame_id is empty")) {
        return false;
    }
    if (!check(
            frame.image.width == kCameraWidth && frame.image.height == kCameraHeight,
            "camera resolution is wrong")) {
        return false;
    }
    if (!check(!frame.image.data.empty(), "camera image data is empty")) return false;
    if (!check(
            frame.image.data.size() ==
                static_cast<std::size_t>(frame.image.step) * frame.image.height,
            "camera image payload does not match step and height")) {
        return false;
    }
    // The backend reports the calibration image size only for colour streams,
    // so a reported size must at least agree with the image we received.
    if (!check(
            frame.camera_info.width == 0 || frame.camera_info.width == frame.image.width,
            "camera_info width disagrees with the image")) {
        return false;
    }
    if (!check(
            frame.camera_info.height == 0 || frame.camera_info.height == frame.image.height,
            "camera_info height disagrees with the image")) {
        return false;
    }
    return check(
        frame.camera_info.k[0] > 0.0 && frame.camera_info.k[4] > 0.0,
        "camera intrinsics are empty");
}

bool check_cameras(RobotHardware& robot) {
    std::array<bool, kCameraCount> ready{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
    while (std::chrono::steady_clock::now() < deadline) {
        bool all_ready = true;
        for (std::size_t index = 0; index < kCameraCount; ++index) {
            if (ready[index]) continue;
            CameraFrame frame;
            if (!robot.read_state(static_cast<Camera>(index), frame)) {
                std::cerr << "camera " << index << " read failed\n";
                return false;
            }
            if (frame.image.data.empty()) {
                all_ready = false;
                continue;
            }
            if (!check_camera(frame)) {
                std::cerr << "camera " << index << " validation failed\n";
                return false;
            }
            ready[index] = true;
        }
        bool complete = true;
        for (const bool value : ready) complete = complete && value;
        if (complete) return true;
        if (!advance(robot, 25)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    for (std::size_t index = 0; index < kCameraCount; ++index) {
        if (!ready[index]) std::cerr << "camera " << index << " never produced a frame\n";
    }
    return false;
}

}  // namespace

int main() {
    if (!check_invalid_control_periods()) return EXIT_FAILURE;

    RobotHardware robot;
    if (!check_uninitialized_rejections(robot)) return EXIT_FAILURE;
    if (!check(robot.initialize(make_options()), "initialize failed")) return EXIT_FAILURE;
    if (!check(!robot.initialize(make_options()), "double initialize was accepted")) {
        return EXIT_FAILURE;
    }
    RobotState initial;
    const bool initial_ok = robot.activate() && robot.read_state(initial);
    if (!initial_ok) {
        std::cerr << "activation failed\n";
        return EXIT_FAILURE;
    }
    if (!check(!robot.activate(), "double activate was accepted")) return EXIT_FAILURE;
    // Re-enter from a clean activate() to exercise the safe command path.
    if (!check(robot.deactivate(), "deactivate failed")) return EXIT_FAILURE;
    if (!check(!robot.deactivate(), "double deactivate was accepted")) return EXIT_FAILURE;
    if (!check_safe_activation(robot, initial)) return EXIT_FAILURE;
    if (!check_motion_snapshot(robot)) return EXIT_FAILURE;
    if (!check_arm_control(robot)) return EXIT_FAILURE;
    if (!check_spine_and_tmr(robot)) return EXIT_FAILURE;
    if (!check_grippers(robot)) return EXIT_FAILURE;
    if (!check_atomic_command_rejection(robot)) return EXIT_FAILURE;
    if (!check_imu(robot)) return EXIT_FAILURE;
    if (!check_lidar(robot, Lidar::Front)) return EXIT_FAILURE;
    if (!check_lidar(robot, Lidar::Rear)) return EXIT_FAILURE;
    if (!check_invalid_device_ids(robot)) return EXIT_FAILURE;
    if (!check_cameras(robot)) return EXIT_FAILURE;
    // shutdown() must safe-stop on its own, without a preceding deactivate().
    if (!check(robot.shutdown(), "shutdown from Active failed")) return EXIT_FAILURE;
    if (!check(robot.shutdown(), "repeated shutdown failed")) return EXIT_FAILURE;
    // The state machine must be back at Uninitialized.
    if (!check_uninitialized_rejections(robot)) return EXIT_FAILURE;

    // Reinitialize after shutdown and smoke test the second lifecycle.
    if (!check(robot.initialize(make_options()), "reinitialize failed")) return EXIT_FAILURE;
    RobotState reinitialized;
    if (!check(
            robot.activate() && robot.read_state(reinitialized),
            "reactivation after reinitialize failed")) {
        return EXIT_FAILURE;
    }
    if (!check(motion_state_is_finite(reinitialized), "state after reinitialize is not finite")) {
        return EXIT_FAILURE;
    }
    if (!check(robot.deactivate() && robot.shutdown(), "shutdown after reinitialize failed")) {
        return EXIT_FAILURE;
    }
    std::cout << "RobotHardware: lifecycle, motion, sensors and atomicity passed\n";
    return EXIT_SUCCESS;
}
