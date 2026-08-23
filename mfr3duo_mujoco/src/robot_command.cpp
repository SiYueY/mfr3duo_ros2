#include "robot_command.hpp"

#include "component_ids.hpp"
#include "mujoco_simulation/component/joint.hpp"

namespace mfr3duo_mujoco {
namespace {

mujoco_simulation::JointMode lower(JointMode mode) {
    switch (mode) {
        case JointMode::Position:
            return mujoco_simulation::JointMode::Position;
        case JointMode::Velocity:
            return mujoco_simulation::JointMode::Velocity;
        case JointMode::Effort:
            return mujoco_simulation::JointMode::Effort;
        case JointMode::Hybrid:
            return mujoco_simulation::JointMode::Hybrid;
        default:
            return mujoco_simulation::JointMode::None;
    }
}

mujoco_simulation::JointCommand command(std::size_t id, const JointCommand& in) {
    mujoco_simulation::JointCommand out;
    out.id = id;
    out.mode = static_cast<std::uint8_t>(lower(in.mode));
    out.position = in.position;
    out.velocity = in.velocity;
    out.effort = in.effort;
    out.stiffness = in.stiffness;
    out.damping = in.damping;
    return out;
}
}  // namespace

bool to_runtime_command(const RobotCommand& input, mujoco_simulation::RobotCommand& output) {
    output = {};
    output.joints.reserve(kActiveJointIds.size());
    for (std::size_t i = 0; i < 7; ++i)
        output.joints.push_back(command(kLeftArmFirst + i, input.left_arm.joints[i]));
    for (std::size_t i = 0; i < 7; ++i)
        output.joints.push_back(command(kRightArmFirst + i, input.right_arm.joints[i]));
    output.joints.push_back(command(kSpine, input.spine.joint));
    JointCommand steer;
    steer.mode = JointMode::Position;
    steer.position = input.tmr.front_steering_position;
    output.joints.push_back(command(kFrontSteering, steer));
    JointCommand drive;
    drive.mode = JointMode::Velocity;
    drive.velocity = input.tmr.front_drive_velocity;
    output.joints.push_back(command(kFrontDrive, drive));
    steer.position = input.tmr.rear_steering_position;
    output.joints.push_back(command(kRearSteering, steer));
    drive.velocity = input.tmr.rear_drive_velocity;
    output.joints.push_back(command(kRearDrive, drive));
    output.joints.push_back(command(kLeftFinger1, input.left_gripper.actuator));
    output.joints.push_back(command(kRightFinger1, input.right_gripper.actuator));
    return output.joints.size() == kActiveJointIds.size();
}
}  // namespace mfr3duo_mujoco
