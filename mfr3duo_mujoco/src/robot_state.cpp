#include "robot_state.hpp"

#include "component_ids.hpp"

namespace mfr3duo_mujoco {
namespace {
JointMode upper(std::uint8_t mode) {
    switch (static_cast<mujoco_simulation::JointMode>(mode)) {
        case mujoco_simulation::JointMode::Position:
            return JointMode::Position;
        case mujoco_simulation::JointMode::Velocity:
            return JointMode::Velocity;
        case mujoco_simulation::JointMode::Effort:
            return JointMode::Effort;
        case mujoco_simulation::JointMode::Hybrid:
            return JointMode::Hybrid;
        default:
            return JointMode::None;
    }
}
bool get(const mujoco_simulation::JointStates& states, std::size_t id, JointState& output) {
    if (!states || id >= states->size() || !(*states)[id] || (*states)[id]->id != id) return false;
    const auto& in = *(*states)[id];
    output = {upper(in.mode), in.position, in.velocity, in.effort};
    return true;
}
}  // namespace

bool from_runtime_state(const mujoco_simulation::RobotState& input, RobotState& output) {
    if (!input.joints || !input.imus || input.imus->empty() || !(*input.imus)[0]) return false;
    RobotState value;
    value.time = input.simulation_time;
    value.step_count = input.step;
    value.tmr.base = BaseState{};
    for (std::size_t i = 0; i < 7; ++i)
        if (!get(input.joints, kLeftArmFirst + i, value.left_arm.joints[i]) ||
            !get(input.joints, kRightArmFirst + i, value.right_arm.joints[i]))
            return false;
    if (!get(input.joints, kSpine, value.spine.joint) ||
        !get(input.joints, kFrontSteering, value.tmr.front_steering) ||
        !get(input.joints, kFrontDrive, value.tmr.front_drive) ||
        !get(input.joints, kRearSteering, value.tmr.rear_steering) ||
        !get(input.joints, kRearDrive, value.tmr.rear_drive) ||
        !get(input.joints, kRocker, value.tmr.rocker) ||
        !get(input.joints, kFrontCasterSteering, value.tmr.front_caster_steering) ||
        !get(input.joints, kFrontCasterRolling, value.tmr.front_caster_rolling) ||
        !get(input.joints, kRearCasterSteering, value.tmr.rear_caster_steering) ||
        !get(input.joints, kRearCasterRolling, value.tmr.rear_caster_rolling) ||
        !get(input.joints, kLeftFinger1, value.left_gripper.left_finger) ||
        !get(input.joints, kLeftFinger2, value.left_gripper.right_finger) ||
        !get(input.joints, kRightFinger1, value.right_gripper.left_finger) ||
        !get(input.joints, kRightFinger2, value.right_gripper.right_finger))
        return false;
    const auto& imu = *(*input.imus)[0];
    value.imu.timestamp = imu.timestamp;
    value.imu.orientation = {
        imu.orientation[3], imu.orientation[0], imu.orientation[1], imu.orientation[2]};
    value.imu.angular_velocity = {
        imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]};
    value.imu.linear_acceleration = {
        imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]};
    output = std::move(value);
    return true;
}
}  // namespace mfr3duo_mujoco
