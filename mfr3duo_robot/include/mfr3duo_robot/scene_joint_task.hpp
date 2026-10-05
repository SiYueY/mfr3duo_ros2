#pragma once
#include "mfr3duo_robot/task.hpp"
#include <utility>

namespace mfr3duo_robot {
/** Simulation fixture actuator; it does not claim an arm grasp or handle skill. */
class SceneJointTask final : public RobotTask {
public:
    SceneJointTask(std::string name, double position)
    : name_(std::move(name)), position_(position) {}
    const std::string& joint_name() const noexcept { return name_; }
    double position() const noexcept { return position_; }
    std::string_view name() const noexcept override { return "SceneJointTask"; }
    std::unique_ptr<RobotTask> clone() const override {
        return std::make_unique<SceneJointTask>(*this);
    }

private:
    std::string name_;
    double position_;
};
}  // namespace mfr3duo_robot
