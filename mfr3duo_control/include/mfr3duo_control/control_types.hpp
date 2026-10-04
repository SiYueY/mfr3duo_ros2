#pragma once

#include <cstdint>
#include <string>

#include "rclcpp/time.hpp"

namespace mfr3duo_control {
enum class Arm : std::uint8_t { Left, Right };
enum class Gripper : std::uint8_t { Left, Right };
struct BaseVelocity {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};
enum class ErrorCode {
    Success,
    NotInitialized,
    NotReady,
    InvalidArgument,
    StateUnavailable,
    StateStale,
    Busy,
    PreviousOperationNotTerminated,
    ControllerUnavailable,
    GoalRejected,
    ExecutionFailed,
    Timeout,
    Canceled,
    CancelFailed,
    InternalError,
};
enum class OperationState { Idle, WaitingForGoalResponse, Active, Canceling, TerminationUnknown };
struct Result {
    ErrorCode code{ErrorCode::Success};
    std::string message;
    explicit operator bool() const noexcept { return code == ErrorCode::Success; }
};
template <typename T>
struct StateResult {
    Result result;
    T value{};
    rclcpp::Time stamp;
    explicit operator bool() const noexcept { return static_cast<bool>(result); }
};
}  // namespace mfr3duo_control
