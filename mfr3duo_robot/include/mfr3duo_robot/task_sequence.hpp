#pragma once
#include "mfr3duo_robot/task.hpp"
#include <vector>
namespace mfr3duo_robot {
class TaskSequence final : public RobotTask {
public:
    void add(std::unique_ptr<RobotTask> task) { tasks_.push_back(std::move(task)); }
    const std::vector<std::unique_ptr<RobotTask>>& tasks() const noexcept { return tasks_; }
    std::string_view name() const noexcept override { return "TaskSequence"; }
    std::unique_ptr<RobotTask> clone() const override;

private:
    std::vector<std::unique_ptr<RobotTask>> tasks_;
};
}  // namespace mfr3duo_robot
