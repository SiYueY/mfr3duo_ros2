#include "mfr3duo_robot/navigate_task.hpp"
#include "mfr3duo_robot/pick_task.hpp"
#include "mfr3duo_robot/place_task.hpp"
#include "mfr3duo_robot/task_sequence.hpp"
namespace mfr3duo_robot {
std::unique_ptr<RobotTask> NavigateTask::clone() const {
    return std::make_unique<NavigateTask>(*this);
}
std::unique_ptr<RobotTask> PickTask::clone() const { return std::make_unique<PickTask>(*this); }
std::unique_ptr<RobotTask> PlaceTask::clone() const { return std::make_unique<PlaceTask>(*this); }
std::unique_ptr<RobotTask> TaskSequence::clone() const {
    auto copy = std::make_unique<TaskSequence>();
    if (timeout()) copy->set_timeout(*timeout());
    for (const auto& task : tasks_) copy->add(task ? task->clone() : nullptr);
    return copy;
}
}  // namespace mfr3duo_robot
