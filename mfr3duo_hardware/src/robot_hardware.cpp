#include "mfr3duo_hardware/robot_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

#include "mfr3duo_mujoco/simulation.hpp"

namespace mfr3duo_hardware {
namespace {

enum class Lifecycle {
    Uninitialized,
    Inactive,
    Active,
};

bool finite(double value) { return std::isfinite(value) != 0; }

bool valid_mode(JointControlMode mode) {
    switch (mode) {
        case JointControlMode::Position:
        case JointControlMode::Velocity:
        case JointControlMode::Effort:
            return true;
    }
    return false;
}

bool valid_joint(const JointCommand& joint) {
    return finite(joint.position) && finite(joint.velocity) && finite(joint.effort);
}

bool valid_arm(const ArmCommand& arm) {
    if (!valid_mode(arm.mode)) return false;
    for (const auto& joint : arm.joints) {
        if (!valid_joint(joint)) return false;
    }
    return true;
}

bool valid_command(const RobotCommand& command) {
    return valid_arm(command.left_arm) && valid_arm(command.right_arm) &&
           finite(command.spine.position) && finite(command.tmr.front_steering_position) &&
           finite(command.tmr.front_drive_velocity) && finite(command.tmr.rear_steering_position) &&
           finite(command.tmr.rear_drive_velocity) && finite(command.left_gripper.width) &&
           finite(command.left_gripper.velocity) && finite(command.left_gripper.effort) &&
           finite(command.right_gripper.width) && finite(command.right_gripper.velocity) &&
           finite(command.right_gripper.effort);
}

// Device id mapping rejects out-of-range enum values instead of silently
// falling back to another device.
bool backend_lidar(Lidar id, mfr3duo_mujoco::Lidar& output) {
    switch (id) {
        case Lidar::Front:
            output = mfr3duo_mujoco::Lidar::Front;
            return true;
        case Lidar::Rear:
            output = mfr3duo_mujoco::Lidar::Rear;
            return true;
    }
    return false;
}

// The documented camera order is not the backend enum order, so the mapping is
// explicit instead of a cast.
bool backend_camera(Camera id, mfr3duo_mujoco::Camera& output) {
    switch (id) {
        case Camera::FrontColor:
            output = mfr3duo_mujoco::Camera::FrontColor;
            return true;
        case Camera::FrontDepth:
            output = mfr3duo_mujoco::Camera::FrontDepth;
            return true;
        case Camera::RearColor:
            output = mfr3duo_mujoco::Camera::RearColor;
            return true;
        case Camera::RearDepth:
            output = mfr3duo_mujoco::Camera::RearDepth;
            return true;
        case Camera::LeftColor:
            output = mfr3duo_mujoco::Camera::LeftColor;
            return true;
        case Camera::LeftDepth:
            output = mfr3duo_mujoco::Camera::LeftDepth;
            return true;
        case Camera::RightColor:
            output = mfr3duo_mujoco::Camera::RightColor;
            return true;
        case Camera::RightDepth:
            output = mfr3duo_mujoco::Camera::RightDepth;
            return true;
        case Camera::LeftWristColor:
            output = mfr3duo_mujoco::Camera::LeftWristColor;
            return true;
        case Camera::LeftWristDepth:
            output = mfr3duo_mujoco::Camera::LeftWristDepth;
            return true;
        case Camera::RightWristColor:
            output = mfr3duo_mujoco::Camera::RightWristColor;
            return true;
        case Camera::RightWristDepth:
            output = mfr3duo_mujoco::Camera::RightWristDepth;
            return true;
        case Camera::HeadZedLeft:
            output = mfr3duo_mujoco::Camera::HeadZedLeft;
            return true;
        case Camera::HeadZedRight:
            output = mfr3duo_mujoco::Camera::HeadZedRight;
            return true;
    }
    return false;
}

mfr3duo_mujoco::JointControlMode backend_mode(JointControlMode mode) {
    switch (mode) {
        case JointControlMode::Position:
            return mfr3duo_mujoco::JointControlMode::Position;
        case JointControlMode::Velocity:
            return mfr3duo_mujoco::JointControlMode::Velocity;
        case JointControlMode::Effort:
            return mfr3duo_mujoco::JointControlMode::Effort;
    }
    return mfr3duo_mujoco::JointControlMode::Position;
}

std::uint64_t to_nanoseconds(double seconds) {
    return static_cast<std::uint64_t>(std::llround(seconds * 1.0e9));
}

void copy_joint_state(const mfr3duo_mujoco::JointState& source, JointState& target) {
    target.position = source.position;
    target.velocity = source.velocity;
    target.effort = source.effort;
}

void copy_arm_state(const mfr3duo_mujoco::ArmState& source, ArmState& target) {
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        copy_joint_state(source.joints[index], target.joints[index]);
    }
}

void copy_tmr_state(const mfr3duo_mujoco::TmrState& source, TmrState& target) {
    copy_joint_state(source.front_steering, target.front_steering);
    copy_joint_state(source.front_drive, target.front_drive);
    copy_joint_state(source.rear_steering, target.rear_steering);
    copy_joint_state(source.rear_drive, target.rear_drive);
}

void copy_gripper_state(const mfr3duo_mujoco::GripperState& source, GripperState& target) {
    target.width = source.width;
    target.velocity = source.velocity;
    target.effort = source.effort;
    target.stalled = source.stalled;
}

void copy_motion_state(const mfr3duo_mujoco::RobotState& source, RobotState& target) {
    target.sequence = source.sequence;
    target.timestamp_ns = source.timestamp;
    copy_tmr_state(source.tmr, target.tmr);
    target.spine.position = source.spine.position;
    target.spine.velocity = source.spine.velocity;
    copy_arm_state(source.left_arm, target.left_arm);
    copy_arm_state(source.right_arm, target.right_arm);
    copy_gripper_state(source.left_gripper, target.left_gripper);
    copy_gripper_state(source.right_gripper, target.right_gripper);
}

void copy_arm_command(const ArmCommand& source, mfr3duo_mujoco::ArmCommand& target) {
    const auto mode = backend_mode(source.mode);
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        target.joints[index].mode = mode;
        target.joints[index].position = source.joints[index].position;
        target.joints[index].velocity = source.joints[index].velocity;
        target.joints[index].effort = source.joints[index].effort;
    }
}

mfr3duo_mujoco::RobotCommand to_backend(const RobotCommand& source) {
    mfr3duo_mujoco::RobotCommand target;
    copy_arm_command(source.left_arm, target.left_arm);
    copy_arm_command(source.right_arm, target.right_arm);
    target.spine.mode = mfr3duo_mujoco::JointControlMode::Position;
    target.spine.position = source.spine.position;
    target.tmr.front_steering_position = source.tmr.front_steering_position;
    target.tmr.front_drive_velocity = source.tmr.front_drive_velocity;
    target.tmr.rear_steering_position = source.tmr.rear_steering_position;
    target.tmr.rear_drive_velocity = source.tmr.rear_drive_velocity;
    target.left_gripper.width = source.left_gripper.width;
    target.left_gripper.velocity = source.left_gripper.velocity;
    target.left_gripper.effort = source.left_gripper.effort;
    target.right_gripper.width = source.right_gripper.width;
    target.right_gripper.velocity = source.right_gripper.velocity;
    target.right_gripper.effort = source.right_gripper.effort;
    return target;
}

void copy_imu_state(const mfr3duo_mujoco::ImuState& source, ImuState& target) {
    target.sequence = source.sequence;
    target.timestamp_ns = to_nanoseconds(source.timestamp);
    target.orientation = {
        source.orientation.x, source.orientation.y, source.orientation.z, source.orientation.w};
    target.angular_velocity = {
        source.angular_velocity.x, source.angular_velocity.y, source.angular_velocity.z};
    target.linear_acceleration = {
        source.linear_acceleration.x, source.linear_acceleration.y, source.linear_acceleration.z};
}

void copy_laser_scan(const mfr3duo_mujoco::LaserScan& source, LaserScan& target) {
    target.sequence = source.sequence;
    target.timestamp_ns = source.timestamp;
    target.frame_id = source.frame_id;
    target.angle_min = source.angle_min;
    target.angle_max = source.angle_max;
    target.angle_increment = source.angle_increment;
    target.time_increment = source.time_increment;
    target.scan_time = source.scan_time;
    target.range_min = source.range_min;
    target.range_max = source.range_max;
    target.ranges = source.ranges;
    target.intensities = source.intensities;
}

void copy_image(const mfr3duo_mujoco::Image& source, Image& target) {
    target.width = source.width;
    target.height = source.height;
    target.step = source.step;
    target.encoding = source.encoding;
    target.is_bigendian = source.is_bigendian != 0;
    target.data = source.data;
}

void copy_camera_info(const mfr3duo_mujoco::CameraInfo& source, CameraInfo& target) {
    target.width = source.width;
    target.height = source.height;
    target.distortion_model = source.distortion_model;
    target.d = source.d;
    target.k = source.k;
    target.r = source.r;
    target.p = source.p;
    target.binning_x = source.binning_x;
    target.binning_y = source.binning_y;
}

// Depth streams carry their payload in the backend depth image; colour streams
// carry it in the colour image. Both collapse onto CameraFrame::image.
void copy_camera_frame(const mfr3duo_mujoco::CameraFrame& source, CameraFrame& target) {
    target.sequence = source.sequence;
    target.timestamp_ns = source.timestamp;
    target.frame_id = source.frame_id;
    target.optical_frame_id = source.optical_frame_id;
    copy_image(source.image.data.empty() ? source.depth_image : source.image, target.image);
    copy_camera_info(source.camera_info, target.camera_info);
}

// Hold the current pose with zero velocity and zero effort.
RobotCommand safe_command(const RobotState& state) {
    RobotCommand command;
    command.left_arm.mode = JointControlMode::Position;
    command.right_arm.mode = JointControlMode::Position;
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
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

struct RobotHardware::Impl {
    /** @brief Read the lifecycle without racing the sensor thread. */
    Lifecycle lifecycle() const {
        const std::lock_guard<std::mutex> lock(state_mutex);
        return current_lifecycle;
    }

    void set_lifecycle(Lifecycle value) {
        const std::lock_guard<std::mutex> lock(state_mutex);
        current_lifecycle = value;
    }

    bool is_active() const { return lifecycle() == Lifecycle::Active; }

    /** @brief Locked copy of the published motion snapshot. */
    RobotState cached_state() const {
        const std::lock_guard<std::mutex> lock(state_mutex);
        return state;
    }

    /** @brief Locked copy of the snapshot, valid only while Active. */
    bool snapshot(RobotState& output) const {
        const std::lock_guard<std::mutex> lock(state_mutex);
        if (current_lifecycle != Lifecycle::Active) return false;
        output = state;
        return true;
    }

    void publish(const RobotState& next) {
        const std::lock_guard<std::mutex> lock(state_mutex);
        state = next;
    }

    /** @brief Read the backend outside the lock and publish the new snapshot. */
    bool refresh_state();

    bool submit(const RobotCommand& command);

    mfr3duo_mujoco::Simulation simulation;
    std::size_t steps_per_update{1};

    // Guards the published motion snapshot and the lifecycle so the control
    // thread and the sensor thread never touch them concurrently. Backend calls
    // (step, write_command, sensor reads) always happen outside this lock.
    mutable std::mutex state_mutex;
    Lifecycle current_lifecycle{Lifecycle::Uninitialized};
    RobotState state;
};

bool RobotHardware::Impl::refresh_state() {
    mfr3duo_mujoco::RobotState backend_state;
    if (!simulation.read_state(backend_state)) return false;
    RobotState next;
    copy_motion_state(backend_state, next);
    publish(next);
    return true;
}

bool RobotHardware::Impl::submit(const RobotCommand& command) {
    if (!valid_command(command)) return false;
    return simulation.write_command(to_backend(command));
}

RobotHardware::RobotHardware() : impl_(std::make_unique<Impl>()) {}

// The destructor goes through the public lifecycle instead of bypassing it.
RobotHardware::~RobotHardware() { shutdown(); }

RobotHardware::RobotHardware(RobotHardware&&) noexcept = default;

RobotHardware& RobotHardware::operator=(RobotHardware&&) noexcept = default;

bool RobotHardware::initialize(const RobotHardwareOptions& options) {
    if (impl_ == nullptr || impl_->lifecycle() != Lifecycle::Uninitialized) return false;
    const auto control_period = options.control_period;
    if (control_period <= std::chrono::nanoseconds::zero()) return false;

    mfr3duo_mujoco::SimulationOptions backend;
    backend.viewer_enabled = options.viewer_enabled;
    backend.initial_keyframe = options.initial_keyframe;
    for (const auto& mapping : options.grasp_objects)
        backend.grasp_objects.push_back(
            {mapping.object_id, mapping.body_name, mapping.collision_geom});
    backend.cameras_enabled = true;
    backend.lidars_enabled = true;
    backend.imu_enabled = true;
    backend.camera_width = 320;
    backend.camera_height = 180;
    backend.camera_period = 0.04;
    const bool initialized = options.model_path.empty()
                                 ? impl_->simulation.initialize(backend)
                                 : impl_->simulation.initialize(options.model_path, backend);
    if (!initialized) return false;

    // Take the physics period from the backend instead of duplicating it here so
    // the backend stays the single source of truth. This advances the simulation
    // by one physics step before activate(), which then holds the current pose.
    const double time_before = impl_->simulation.time();
    if (!impl_->simulation.step(1)) {
        impl_->simulation.shutdown();
        return false;
    }
    const auto physics_period = std::chrono::nanoseconds(
        static_cast<std::int64_t>(std::llround((impl_->simulation.time() - time_before) * 1.0e9)));
    const bool period_usable = physics_period > std::chrono::nanoseconds::zero() &&
                               control_period.count() % physics_period.count() == 0;
    if (!period_usable) {
        impl_->simulation.shutdown();
        return false;
    }
    const auto steps = static_cast<std::size_t>(control_period.count() / physics_period.count());
    if (steps == 0) {
        impl_->simulation.shutdown();
        return false;
    }
    if (!impl_->refresh_state()) {
        impl_->simulation.shutdown();
        return false;
    }
    impl_->steps_per_update = steps;
    impl_->set_lifecycle(Lifecycle::Inactive);
    return true;
}

bool RobotHardware::activate() {
    if (impl_ == nullptr || impl_->lifecycle() != Lifecycle::Inactive) return false;
    if (!impl_->refresh_state()) return false;
    if (!impl_->submit(safe_command(impl_->cached_state()))) return false;
    impl_->set_lifecycle(Lifecycle::Active);
    return true;
}

bool RobotHardware::deactivate() {
    if (impl_ == nullptr || impl_->lifecycle() != Lifecycle::Active) return false;
    if (!impl_->refresh_state()) return false;
    if (!impl_->submit(safe_command(impl_->cached_state()))) return false;
    impl_->set_lifecycle(Lifecycle::Inactive);
    return true;
}

bool RobotHardware::shutdown() {
    if (impl_ == nullptr) return false;
    const auto current = impl_->lifecycle();
    if (current == Lifecycle::Uninitialized) return true;

    // Best effort safe stop. A failed hold must never keep the object alive, so
    // the shutdown always continues and always ends in Uninitialized.
    bool stopped = true;
    if (current == Lifecycle::Active) {
        stopped = impl_->refresh_state() && impl_->submit(safe_command(impl_->cached_state()));
    }

    const bool closed = impl_->simulation.shutdown();
    impl_->set_lifecycle(Lifecycle::Uninitialized);
    impl_->steps_per_update = 1;
    impl_->publish(RobotState{});
    return stopped && closed;
}

bool RobotHardware::update() {
    if (impl_ == nullptr || !impl_->is_active()) return false;
    if (!impl_->simulation.step(impl_->steps_per_update)) return false;
    return impl_->refresh_state();
}

bool RobotHardware::write_command(const RobotCommand& command) {
    if (impl_ == nullptr || !impl_->is_active()) return false;
    return impl_->submit(command);
}

bool RobotHardware::read_state(RobotState& state) const {
    if (impl_ == nullptr) return false;
    return impl_->snapshot(state);
}

bool RobotHardware::observe_grasp(
    const std::string& object_id, GraspManipulator hand, GraspObservation& observation) const {
    observation = {};
    if (impl_ == nullptr || !impl_->is_active()) {
        observation.diagnostic = "hardware not active";
        return false;
    }
    mfr3duo_mujoco::Gripper backend_hand;
    if (hand == GraspManipulator::Left)
        backend_hand = mfr3duo_mujoco::Gripper::Left;
    else if (hand == GraspManipulator::Right)
        backend_hand = mfr3duo_mujoco::Gripper::Right;
    else {
        observation.diagnostic = "invalid manipulator";
        return false;
    }
    mfr3duo_mujoco::GraspObservation source;
    const bool valid = impl_->simulation.observe_grasp(object_id, backend_hand, source);
    observation.valid = source.valid;
    observation.object_visible = source.object_visible;
    observation.left_finger_contact = source.left_finger_contact;
    observation.right_finger_contact = source.right_finger_contact;
    observation.sequence = source.sequence;
    observation.timestamp_ns = to_nanoseconds(source.timestamp);
    observation.object_position = {
        source.object_pose.position.x, source.object_pose.position.y,
        source.object_pose.position.z};
    observation.object_orientation = {
        source.object_pose.orientation.x, source.object_pose.orientation.y,
        source.object_pose.orientation.z, source.object_pose.orientation.w};
    observation.tool_position = {
        source.tool_pose.position.x, source.tool_pose.position.y, source.tool_pose.position.z};
    observation.tool_orientation = {
        source.tool_pose.orientation.x, source.tool_pose.orientation.y,
        source.tool_pose.orientation.z, source.tool_pose.orientation.w};
    observation.diagnostic = std::move(source.diagnostic);
    return valid;
}

bool RobotHardware::read_state(ImuState& state) const {
    if (impl_ == nullptr || !impl_->is_active()) return false;
    // The backend fills frame_id with a string longer than the small-string
    // capacity of libstdc++, so a fresh buffer would allocate on every control
    // cycle. A per-thread scratch buffer keeps the control path allocation free.
    thread_local mfr3duo_mujoco::ImuState scratch;
    if (!impl_->simulation.read_state(scratch)) return false;
    copy_imu_state(scratch, state);
    return true;
}

bool RobotHardware::read_state(PassiveJointStates& state) const {
    if (impl_ == nullptr || !impl_->is_active()) return false;
    mfr3duo_mujoco::TmrPassiveState source;
    if (!impl_->simulation.read_state(source)) return false;
    PassiveJointStates result;
    result.timestamp_ns = to_nanoseconds(source.timestamp);
    result.joints = {
        {{source.front_caster_steering.position, source.front_caster_steering.velocity},
         {source.front_caster_wheel.position, source.front_caster_wheel.velocity},
         {source.rocker_arm.position, source.rocker_arm.velocity},
         {source.rear_caster_steering.position, source.rear_caster_steering.velocity},
         {source.rear_caster_wheel.position, source.rear_caster_wheel.velocity}}};
    state = result;
    return true;
}

bool RobotHardware::read_state(BasePoseState& state) const {
    if (impl_ == nullptr || !impl_->is_active()) return false;
    mfr3duo_mujoco::BasePoseState source;
    if (!impl_->simulation.read_state(source)) return false;
    BasePoseState result;
    result.sequence = source.sequence;
    result.timestamp_ns = to_nanoseconds(source.timestamp);
    result.position = {source.pose.position.x, source.pose.position.y, source.pose.position.z};
    result.orientation = {
        source.pose.orientation.x, source.pose.orientation.y, source.pose.orientation.z,
        source.pose.orientation.w};
    state = result;
    return true;
}

bool RobotHardware::read_state(Lidar id, LaserScan& scan) const {
    if (impl_ == nullptr || !impl_->is_active()) return false;
    mfr3duo_mujoco::Lidar backend_id{};
    if (!backend_lidar(id, backend_id)) return false;
    mfr3duo_mujoco::LaserScan source;
    if (!impl_->simulation.read_state(backend_id, source)) return false;
    copy_laser_scan(source, scan);
    return true;
}

bool RobotHardware::read_state(Camera id, CameraFrame& frame) const {
    if (impl_ == nullptr || !impl_->is_active()) return false;
    mfr3duo_mujoco::Camera backend_id{};
    if (!backend_camera(id, backend_id)) return false;
    mfr3duo_mujoco::CameraFrame source;
    if (!impl_->simulation.read_state(backend_id, source)) return false;
    copy_camera_frame(source, frame);
    return true;
}

}  // namespace mfr3duo_hardware
