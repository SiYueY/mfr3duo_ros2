#include <chrono>
#include <iostream>
#include <stdexcept>
#include <type_traits>
#include "mfr3duo_robot/robot.hpp"
using namespace mfr3duo_robot;
using namespace std::chrono_literals;
namespace {
void require(bool value, const char* text) {
    if (!value) throw std::runtime_error(text);
}
}  // namespace
int main(int argc, char** argv) {
    static_assert(!std::is_copy_constructible_v<Robot> && !std::is_move_constructible_v<Robot>);
    static_assert(std::is_copy_constructible_v<TaskHandle>);
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        TaskHandle invalid;
        require(
            !invalid.valid() && invalid.wait().error == TaskError::InvalidTask &&
                invalid.cancel().error == TaskError::InvalidTask && !invalid.result(),
            "invalid Handle contract");
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "simulation_world";
        pose.pose.orientation.w = 1;
        auto pick = std::make_unique<PickTask>("box");
        pick->set_manipulator(Manipulator::Left);
        pick->set_grasp_pose(pose);
        auto* original = pick.get();
        TaskSequence sequence;
        sequence.set_timeout(2s);
        sequence.add(std::move(pick));
        auto nested = std::make_unique<TaskSequence>();
        auto navigation_pose = pose;
        navigation_pose.header.frame_id = "map";
        nested->add(std::make_unique<NavigateTask>(navigation_pose));
        sequence.add(std::move(nested));
        auto cloned = sequence.clone();
        original->set_manipulator(Manipulator::Right);
        pose.pose.position.x = 9;
        original->set_grasp_pose(pose);
        const auto* copy = dynamic_cast<TaskSequence*>(cloned.get());
        const auto* copied_pick = dynamic_cast<PickTask*>(copy->tasks()[0].get());
        require(
            copy && copy->timeout() == 2s && copied_pick && copied_pick != original &&
                copied_pick->manipulator() == Manipulator::Left &&
                copied_pick->grasp_pose()->pose.position.x == 0,
            "Sequence deep snapshot isolation");
        require(
            dynamic_cast<TaskSequence*>(copy->tasks()[1].get())->tasks()[0].get() !=
                dynamic_cast<TaskSequence*>(sequence.tasks()[1].get())->tasks()[0].get(),
            "nested deep clone");
        TaskHandle retained;
        auto node = rclcpp::Node::make_shared("task_input_test");
        {
            Robot robot(node);
            require(
                robot.state() == RobotState::Uninitialized && !robot.is_ready() && !robot.is_busy(),
                "explicit initialization");
            retained = robot.start(std::make_unique<PickTask>("box"));
            require(
                retained.valid() && retained.wait().error == TaskError::RobotNotReady,
                "NotReady retained Handle");
            require(robot.start(nullptr).wait().error == TaskError::InvalidTask, "null input");
            require(
                robot.start(std::make_unique<PickTask>("unknown")).wait().error ==
                    TaskError::InvalidTask,
                "unknown identity");
            require(
                robot.initialize(0ms).error == TaskError::InvalidTask, "nonpositive initialize");
            auto overflowing = std::make_unique<PickTask>("box");
            overflowing->set_timeout(std::chrono::milliseconds::max());
            require(
                robot.start(std::move(overflowing)).wait().error == TaskError::InvalidTask &&
                    robot.initialize(std::chrono::milliseconds::max()).error ==
                        TaskError::InvalidTask,
                "unrepresentable steady deadline accepted");
            const auto initialize_start = std::chrono::steady_clock::now();
            require(
                !robot.initialize(100ms) &&
                    std::chrono::steady_clock::now() - initialize_start < 500ms,
                "initialization exceeded total timeout without an executor/server");
            require(
                robot.start(std::make_unique<NavigateTask>(pose)).wait().error ==
                    TaskError::InvalidTask,
                "invalid navigation frame was not rejected before motion");
            require(
                robot.execute(sequence).error == TaskError::RobotNotReady,
                "synchronous cloned input");
        }
        require(
            retained.result() && retained.wait().error == TaskError::RobotNotReady &&
                retained.cancel().error == TaskError::RobotNotReady,
            "Handle outlives owner");
        node.reset();
        std::cout << "TASK_INPUT_PASS deep clone, ownership, invalid/NotReady, retained Handle; "
                     "Level A only\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
