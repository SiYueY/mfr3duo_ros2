#pragma once
namespace mfr3duo_robot {
enum class RobotState { Uninitialized, Ready, Executing, Error };
enum class TaskState { Pending, Running, Succeeded, Failed, Canceled };
enum class Manipulator { Auto, Left, Right };
enum class TaskError {
    None,
    RobotNotReady,
    RobotBusy,
    PreviousOperationNotTerminated,
    InvalidTask,
    PlanningFailed,
    ExecutionFailed,
    NavigationFailed,
    GraspFailed,
    GraspLost,
    RecoveryRequired,
    Timeout,
    Canceled,
    CancelFailed,
    InternalError
};
}  // namespace mfr3duo_robot
