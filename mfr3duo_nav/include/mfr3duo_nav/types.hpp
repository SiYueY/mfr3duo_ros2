#pragma once

#include <string>

namespace mfr3duo_nav {

enum class NavigationState {
    Idle,
    Running,
    Canceling,
    TerminationUnknown,
    Succeeded,
    Failed,
    Canceled
};

enum class ErrorCode {
    Success,
    NotInitialized,
    NotReady,
    Busy,
    PreviousOperationNotTerminated,
    InvalidGoal,
    StateUnavailable,
    StateStale,
    ServerUnavailable,
    GoalRejected,
    NavigationFailed,
    Timeout,
    Canceled,
    CancelFailed,
    InternalError
};

struct Result {
    ErrorCode code{ErrorCode::Success};
    std::string message;
    explicit operator bool() const noexcept { return code == ErrorCode::Success; }
};

}  // namespace mfr3duo_nav
