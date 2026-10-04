#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "mfr3duo_control/control_types.hpp"
#include "rclcpp/node.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace mfr3duo_control {
// Application owns and spins the executor on another thread during synchronous calls.
class Control final {
public:
    explicit Control(const rclcpp::Node::SharedPtr& node);
    ~Control();
    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;
    Control(Control&&) = delete;
    Control& operator=(Control&&) = delete;
    Result initialize(std::chrono::milliseconds timeout);
    bool is_ready() const;
    Result command_arm_joint_position(
        Arm arm, const std::vector<double>& positions, std::chrono::milliseconds duration);
    Result execute_arm_trajectory(Arm arm, const trajectory_msgs::msg::JointTrajectory& trajectory);
    Result stop_arm(Arm arm);
    Result command_spine_position(double position, std::chrono::milliseconds duration);
    Result stop_spine();
    Result command_gripper(
        Gripper gripper, double finger_position, std::optional<double> max_effort = std::nullopt);
    // Total opening width [m], opening/closing speed [m/s], force [N].
    Result move_gripper(Gripper gripper, double width, double speed);
    Result grasp_gripper(
        Gripper gripper, double width, double speed, double force, double epsilon_inner = 0.005,
        double epsilon_outer = 0.005);
    Result stop_gripper(Gripper gripper);
    Result command_base_velocity(const BaseVelocity& velocity);
    Result stop_base();
    StateResult<std::vector<double>> get_arm_joint_positions(Arm arm) const;
    StateResult<double> get_spine_position() const;
    // Legacy single-finger position [m]; total opening is twice this value.
    StateResult<double> get_gripper_position(Gripper gripper) const;
    StateResult<double> get_gripper_width(Gripper gripper) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace mfr3duo_control
