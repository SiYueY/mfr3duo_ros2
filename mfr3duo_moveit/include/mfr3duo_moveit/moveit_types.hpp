#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace mfr3duo_moveit {
enum class RobotGroup : std::uint8_t { LeftArm, RightArm, Spine };
enum class ErrorCode {
    Success,
    NotInitialized,
    NotReady,
    Busy,
    PreviousOperationNotTerminated,
    InvalidGroup,
    DuplicateGroup,
    UnsupportedCombination,
    GroupNotAdded,
    GroupHasTarget,
    NoTarget,
    UnsupportedTarget,
    TargetAlreadyExists,
    InvalidTarget,
    InvalidStartState,
    InvalidConstraint,
    IKFailed,
    NoIKSolution,
    GoalSamplingFailed,
    PlanningTimeout,
    PathPlanningFailed,
    ExecutionStartStateMismatch,
    ExecutionServerUnavailable,
    ExecutionFailed,
    InvalidPlan,
    IncompleteCartesianPath,
    Timeout,
    Canceled,
    CancelFailed,
    InternalError
};
enum class OperationState { Idle, WaitingForGoalResponse, Active, Canceling, TerminationUnknown };
struct Result {
    ErrorCode code{ErrorCode::Success};
    std::string message;
    explicit operator bool() const noexcept { return code == ErrorCode::Success; }
};
class MoveGroup;
class Plan {
public:
    Plan() = default;
    bool valid() const noexcept { return valid_; }
    const moveit_msgs::msg::RobotState& start_state() const noexcept { return start_state_; }
    const moveit_msgs::msg::RobotTrajectory& trajectory() const noexcept { return trajectory_; }
    const std::vector<RobotGroup>& groups() const noexcept { return groups_; }
    double planning_time() const noexcept { return planning_time_; }

private:
    friend class MoveGroup;
    moveit_msgs::msg::RobotState start_state_;
    moveit_msgs::msg::RobotTrajectory trajectory_;
    std::vector<RobotGroup> groups_;
    double planning_time_{0};
    bool valid_{false};
    std::optional<double> cartesian_fraction_;
};
struct CartesianPath {
    Plan plan;
    double fraction{0};
};
}  // namespace mfr3duo_moveit
