#pragma once
#include <memory>
#include <optional>
#include "mfr3duo_robot/task_result.hpp"
namespace mfr3duo_robot {
class TaskHandle {
public:
    TaskHandle() = default;
    bool valid() const noexcept;
    TaskState state() const;
    TaskResult cancel();
    TaskResult wait();
    std::optional<TaskResult> result() const;

private:
    friend class Robot;
    struct Impl;
    std::shared_ptr<Impl> impl_;
};
}  // namespace mfr3duo_robot
