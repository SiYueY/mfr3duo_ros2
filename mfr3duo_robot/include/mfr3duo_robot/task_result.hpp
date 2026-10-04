#pragma once
#include <string>
#include "mfr3duo_robot/types.hpp"
namespace mfr3duo_robot {
struct TaskResult {
    TaskState state{TaskState::Pending};
    TaskError error{TaskError::None};
    std::string message;
    explicit operator bool() const noexcept {
        return state == TaskState::Succeeded && error == TaskError::None;
    }
};
}  // namespace mfr3duo_robot
