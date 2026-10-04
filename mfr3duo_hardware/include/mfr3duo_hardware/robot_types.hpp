#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Public data model of RobotHardware.
//
// This header deliberately depends on the C++ standard library only. It must
// never include ROS 2, MuJoCo or mfr3duo_mujoco headers, and it must not expose
// backend types. Motion device types are declared first, sensor types after.

namespace mfr3duo_hardware {

// Standard-C++ simulation observation data; no backend IDs or ROS types.
struct SimulationObjectMapping {
    std::string object_id;
    std::string body_name;
    std::string collision_geom;
};
enum class GraspManipulator : std::uint8_t { Left, Right };
struct GraspObservation {
    bool valid{false};
    bool object_visible{false};
    bool left_finger_contact{false};
    bool right_finger_contact{false};
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};
    std::array<double, 3> object_position{};
    std::array<double, 4> object_orientation{0, 0, 0, 1};
    std::array<double, 3> tool_position{};
    std::array<double, 4> tool_orientation{0, 0, 0, 1};
    std::string diagnostic;
};

// ---------------------------------------------------------------------------
// Motion devices
// ---------------------------------------------------------------------------

inline constexpr std::size_t kArmJointCount = 7;

/** @brief Control mode of one whole FR3 arm. */
enum class JointControlMode : std::uint8_t {
    Position = 0,
    Velocity = 1,
    Effort = 2,
};

/** @brief Command values of one arm joint, interpreted according to the arm mode. */
struct JointCommand {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};

/** @brief Feedback values of one arm or TMR joint. */
struct JointState {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};

/**
 * @brief Command for one seven-axis FR3 arm.
 *
 * The control mode belongs to the arm, so all seven joints always share one
 * mode. Mixed per-joint modes cannot be expressed.
 */
struct ArmCommand {
    JointControlMode mode{JointControlMode::Position};
    std::array<JointCommand, kArmJointCount> joints{};
};

/** @brief State of one seven-axis FR3 arm. */
struct ArmState {
    std::array<JointState, kArmJointCount> joints{};
};

/** @brief Position command for the vertical spine joint. */
struct SpineCommand {
    double position{0.0};
};

/** @brief State of the vertical spine joint. */
struct SpineState {
    double position{0.0};
    double velocity{0.0};
};

/** @brief Targets for the four actuated TMR joints. */
struct TmrCommand {
    double front_steering_position{0.0};
    double front_drive_velocity{0.0};

    double rear_steering_position{0.0};
    double rear_drive_velocity{0.0};
};

/** @brief State of the four actuated TMR joints. */
struct TmrState {
    JointState front_steering;
    JointState front_drive;

    JointState rear_steering;
    JointState rear_drive;
};

/** @brief Command for one Franka Hand. */
struct GripperCommand {
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
};

/** @brief State of one Franka Hand. */
struct GripperState {
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
    bool stalled{false};
};

/**
 * @brief Complete whole-robot motion command.
 *
 * RobotHardware publishes one command path only: a RobotCommand is validated,
 * prepared and committed as a whole. There is no device-level write path.
 */
struct RobotCommand {
    TmrCommand tmr;
    SpineCommand spine;

    ArmCommand left_arm;
    ArmCommand right_arm;

    GripperCommand left_gripper;
    GripperCommand right_gripper;
};

/**
 * @brief Coherent whole-robot motion state from one RobotHardware::update().
 *
 * Sensor samples, TMR passive joints, simulation time and simulation step are
 * read through dedicated APIs and are not part of this snapshot.
 */
struct RobotState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    TmrState tmr;
    SpineState spine;

    ArmState left_arm;
    ArmState right_arm;

    GripperState left_gripper;
    GripperState right_gripper;
};

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

/** @brief Measured position/velocity of one unactuated joint. */
struct PassiveJointState {
    double position{0.0};
    double velocity{0.0};
};

/** @brief Observed unactuated chassis joints, separate from commanded motion devices. */
struct PassiveJointStates {
    std::uint64_t timestamp_ns{0};
    // Front caster steering/wheel, rocker arm, rear caster steering/wheel.
    std::array<PassiveJointState, 5> joints{};
};

/** @brief Three-dimensional vector with Cartesian components. */
struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/** @brief Quaternion in x/y/z/w field order. */
struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

/** @brief Simulator ground truth, with its actual physics sample time. */
struct BasePoseState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};
    Vector3 position;
    Quaternion orientation;
};

/** @brief State reported by the base IMU. */
struct ImuState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
};

/** @brief Canonical MFR3Duo 2D LiDAR devices. */
enum class Lidar : std::uint8_t {
    Front,
    Rear,
};

/** @brief One 2D LiDAR scan sample. */
struct LaserScan {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    std::string frame_id;

    float angle_min{0.0F};
    float angle_max{0.0F};
    float angle_increment{0.0F};

    float time_increment{0.0F};
    float scan_time{0.0F};

    float range_min{0.0F};
    float range_max{0.0F};

    std::vector<float> ranges;
    std::vector<float> intensities;
};

/** @brief Canonical MFR3Duo camera streams. */
enum class Camera : std::uint8_t {
    FrontColor,
    FrontDepth,

    RearColor,
    RearDepth,

    LeftColor,
    LeftDepth,

    RightColor,
    RightDepth,

    LeftWristColor,
    LeftWristDepth,

    RightWristColor,
    RightWristDepth,

    HeadZedLeft,
    HeadZedRight,
};

/** @brief Image payload and transport metadata. */
struct Image {
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t step{0};

    std::string encoding;

    bool is_bigendian{false};

    std::vector<std::uint8_t> data;
};

/** @brief Camera calibration and binning metadata. */
struct CameraInfo {
    std::uint32_t width{0};
    std::uint32_t height{0};

    std::string distortion_model;

    std::vector<double> d;

    std::array<double, 9> k{};
    std::array<double, 9> r{};
    std::array<double, 12> p{};

    std::uint32_t binning_x{0};
    std::uint32_t binning_y{0};
};

/** @brief One camera sample. */
struct CameraFrame {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    std::string frame_id;
    std::string optical_frame_id;

    Image image;
    CameraInfo camera_info;
};

}  // namespace mfr3duo_hardware
