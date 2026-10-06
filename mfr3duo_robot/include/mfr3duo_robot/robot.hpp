#pragma once
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <rclcpp/node.hpp>
#include "mfr3duo_robot/task_handle.hpp"
#include "mfr3duo_robot/navigate_task.hpp"
#include "mfr3duo_robot/pick_task.hpp"
#include "mfr3duo_robot/place_task.hpp"
#include "mfr3duo_robot/task_sequence.hpp"
#include "mfr3duo_robot/scene_joint_task.hpp"
namespace mfr3duo_robot {
// Application spins the supplied node concurrently using a multithreaded executor.
// Robot creates neither a thread nor an executor.
class Robot {
public:
    struct PreflightResult {
        bool feasible{false};
        Manipulator manipulator{Manipulator::Auto};
        TaskError error{TaskError::None};
        std::string message;
    };
    explicit Robot(const rclcpp::Node::SharedPtr& node);
    ~Robot();
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot(Robot&&) = delete;
    Robot& operator=(Robot&&) = delete;
    TaskResult initialize(std::chrono::milliseconds timeout);
    RobotState state() const;
    bool is_ready() const;
    bool is_busy() const;
    // Runs collision-aware planning against fresh measured state without
    // sending a base, arm, spine or gripper command.
    PreflightResult preflight(const RobotTask& task);
    TaskResult execute(const RobotTask& task);
    TaskHandle start(std::unique_ptr<RobotTask> task);
    TaskResult cancel();
    TaskResult stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace mfr3duo_robot
