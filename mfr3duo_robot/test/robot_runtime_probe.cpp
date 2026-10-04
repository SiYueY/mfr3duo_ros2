#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <action_msgs/msg/goal_status_array.hpp>
#include <mfr3duo_control/control.hpp>
#include <mfr3duo_moveit/planning_scene_interface.hpp>
#include <mfr3duo_moveit/move_group.hpp>
#include "mfr3duo_robot/robot.hpp"
#include "grasp_observer.hpp"
using namespace std::chrono_literals;
using namespace mfr3duo_robot;
namespace {
void require(bool value, const std::string& label) {
    if (!value) throw std::runtime_error(label);
}
void require(const TaskResult& value, const std::string& label) {
    if (!value)
        throw std::runtime_error(
            label + ": error=" + std::to_string(static_cast<int>(value.error)) + " " +
            value.message);
}
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_runtime_probe");
    node->declare_parameter("perception.objects_topic_prefix", "/task_observer_proxy/objects");
    node->declare_parameter("perception.tools_topic_prefix", "/task_observer_proxy/tools");
    std::atomic<bool> invalidate{false}, executing{false};
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscribers;
    for (const auto& suffix :
         {std::string("objects/box/pose"), std::string("tools/left/pose"),
          std::string("tools/right/pose")}) {
        auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/task_observer_proxy/" + suffix, 10);
        pose_subscribers.push_back(node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/perception/" + suffix, 10,
            [publisher, &invalidate](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
                if (!invalidate.load()) publisher->publish(*message);
            }));
    }
    auto status = node->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/left_arm_controller/follow_joint_trajectory/_action/status",
        rclcpp::QoS(10).reliable().transient_local(),
        [&](const action_msgs::msg::GoalStatusArray& value) {
            for (const auto& item : value.status_list)
                if (item.status == item.STATUS_EXECUTING) executing = true;
        });
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    std::thread spin([&] { executor.spin(); });
    int code = 0;
    try {
        TaskHandle retained;
        {
            Robot robot(node);
            require(robot.initialize(90s), "whole Robot readiness");
            require(robot.is_ready(), "Robot Ready");
            SimulationGraspObserver observer(node);
            auto initial = observer.observe("box", Manipulator::Left);
            require(initial.valid, "initial actual object observation");
            PlaceTask place("box");
            place.set_place_pose(initial.object_pose);
            // Pausing the application executor proves the Pending ownership/cancellation branch.
            executor.cancel();
            spin.join();
            auto pending = robot.start(std::make_unique<PickTask>("box"));
            require(
                pending.valid() && pending.state() == TaskState::Pending, "Pending async input");
            require(
                robot.start(place.clone()).wait().error == TaskError::RobotBusy,
                "Robot Busy rejection");
            require(pending.cancel().state == TaskState::Canceled, "cancel before dispatch");
            auto deadline_task = std::make_unique<PickTask>("box");
            deadline_task->set_timeout(1ms);
            require(
                robot.start(std::move(deadline_task)).wait().error == TaskError::Timeout,
                "Pending deadline");
            spin = std::thread([&] { executor.spin(); });
            auto picked = robot.start(std::make_unique<PickTask>("box"));
            require(
                robot.start(place.clone()).wait().error == TaskError::RobotBusy,
                "active top-level Busy");
            require(picked.wait(), "public PickTask");
            auto held = observer.observe("box", Manipulator::Left);
            require(
                held.valid &&
                    held.object_pose.pose.position.z > initial.object_pose.pose.position.z + .08,
                "actual Task pick");
            std::cout << "ROBOT_PHYSICAL_PICK_PASS" << std::endl;
            invalidate = true;
            const auto health_deadline = std::chrono::steady_clock::now() + 2s;
            while (robot.state() != RobotState::Error &&
                   std::chrono::steady_clock::now() < health_deadline)
                std::this_thread::sleep_for(10ms);
            require(
                robot.state() == RobotState::Error && !robot.is_ready(),
                "idle held observation loss enters Error");
            require(
                robot.start(place.clone()).wait().error == TaskError::RecoveryRequired,
                "idle physical uncertainty blocks new Task");
            invalidate = false;
            require(robot.initialize(30s), "idle held physical recovery");
            executing = false;
            auto canceled = robot.start(place.clone());
            const auto active_deadline = std::chrono::steady_clock::now() + 10s;
            while (!executing && !canceled.result() &&
                   std::chrono::steady_clock::now() < active_deadline)
                std::this_thread::sleep_for(10ms);
            require(executing, "owned Place arm trajectory became active");
            auto canceled_result = canceled.cancel();
            require(
                canceled_result.state == TaskState::Canceled &&
                    canceled_result.error == TaskError::Canceled,
                "Place exact motion cancellation: " + canceled_result.message);
            held = observer.observe("box", Manipulator::Left);
            require(held.valid && robot.is_ready(), "canceled Place retained actual holding");
            mfr3duo_moveit::MoveGroup execution_health(node);
            const auto execution_initialized = execution_health.initialize(30s);
            require(static_cast<bool>(execution_initialized), execution_initialized.message);
            // Stop the application executor while an owned real arm goal is active.
            // The official stop can reach the independent controller; this client
            // cannot confirm results and fresh measured stop until its executor resumes.
            executing = false;
            auto no_terminal = robot.start(place.clone());
            const auto terminal_active_deadline = std::chrono::steady_clock::now() + 10s;
            while (!executing && !no_terminal.result() &&
                   std::chrono::steady_clock::now() < terminal_active_deadline)
                std::this_thread::sleep_for(10ms);
            require(executing, "terminal-loss fixture has an active owned arm goal");
            executor.cancel();
            const auto unconfirmed = no_terminal.cancel();
            require(
                (unconfirmed.error == TaskError::CancelFailed ||
                 unconfirmed.error == TaskError::RecoveryRequired) &&
                    robot.state() == RobotState::Error,
                "unconfirmed cancellation did not enter Error: " + unconfirmed.message);
            const auto blocked_next = robot.start(place.clone()).wait();
            require(
                blocked_next.error == TaskError::PreviousOperationNotTerminated ||
                    blocked_next.error == TaskError::RecoveryRequired,
                "unconfirmed owned motion permitted a new Task");
            spin.join();
            spin = std::thread([&] { executor.spin(); });
            const auto late_deadline = std::chrono::steady_clock::now() + 5s;
            while (!execution_health.is_ready() && std::chrono::steady_clock::now() < late_deadline)
                std::this_thread::sleep_for(10ms);
            require(
                execution_health.is_ready(),
                "late child results and measured stop did not converge");
            require(robot.initialize(30s), "late terminal and held scene recovery");
            require(
                no_terminal.result() && no_terminal.wait().error == unconfirmed.error,
                "late completion overwrote retained failure");
            std::cout << "ROBOT_TERMINATION_UNKNOWN_PASS blocked until terminal and initialize"
                      << std::endl;
            auto timed = place.clone();
            timed->set_timeout(1s);
            auto timed_result = robot.start(std::move(timed)).wait();
            require(
                timed_result.error == TaskError::Timeout,
                "active Place deadline: " + timed_result.message);
            require(robot.is_ready(), "deadline known held state Ready");
            auto unknown_handle = robot.start(place.clone());
            const auto dispatch_deadline = std::chrono::steady_clock::now() + 1s;
            while (unknown_handle.state() == TaskState::Pending &&
                   std::chrono::steady_clock::now() < dispatch_deadline)
                std::this_thread::sleep_for(1ms);
            require(
                unknown_handle.state() == TaskState::Running, "unknown-state fixture dispatched");
            invalidate = true;
            auto unknown = unknown_handle.wait();
            require(
                unknown.error == TaskError::RecoveryRequired && robot.state() == RobotState::Error,
                "unknown physical state enters Error: " + unknown.message);
            require(
                robot.start(place.clone()).wait().error == TaskError::RecoveryRequired,
                "Error forbids next Task");
            invalidate = false;
            require(robot.initialize(30s), "explicit fresh physical/scene recovery");
            require(robot.execute(place), "public PlaceTask");
            auto released = observer.observe("box", Manipulator::Left);
            require(
                released.valid && std::abs(
                                      released.object_pose.pose.position.z -
                                      initial.object_pose.pose.position.z) < .01,
                "actual Task release on support");
            TaskSequence sequence;
            sequence.add(place.clone());
            sequence.add(std::make_unique<PickTask>("box"));
            auto sequence_result = robot.execute(sequence);
            require(
                sequence_result.error == TaskError::InvalidTask &&
                    sequence_result.message.find("Sequence[0]") != std::string::npos,
                "Sequence stops before remaining Pick");
            released = observer.observe("box", Manipulator::Left);
            require(
                released.valid && std::abs(
                                      released.object_pose.pose.position.z -
                                      initial.object_pose.pose.position.z) < .01,
                "remaining Sequence task was not executed");
            // A test-owned gripper action physically releases the held object while
            // Robot owns a Place arm trajectory; no observation is fabricated.
            require(robot.execute(PickTask("box")), "Pick before actual unexpected release");
            mfr3duo_control::Control disturbance(node);
            require(static_cast<bool>(disturbance.initialize(10s)), "release fixture readiness");
            executing = false;
            auto dropped = robot.start(place.clone());
            const auto drop_active_deadline = std::chrono::steady_clock::now() + 10s;
            while (!executing && !dropped.result() &&
                   std::chrono::steady_clock::now() < drop_active_deadline)
                std::this_thread::sleep_for(10ms);
            require(executing, "unexpected release occurs during actual owned motion");
            require(
                static_cast<bool>(
                    disturbance.command_gripper(mfr3duo_control::Gripper::Left, .04, 20.0)),
                "physical external release");
            std::this_thread::sleep_for(300ms);
            auto lost = dropped.cancel();
            require(
                lost.error == TaskError::GraspLost && robot.is_ready(),
                "actual lost grasp maps to GraspLost: " + lost.message);
            mfr3duo_moveit::PlanningSceneInterface verified_scene(node);
            require(static_cast<bool>(verified_scene.initialize(2s)), "released scene readiness");
            bool still_attached = true;
            require(
                static_cast<bool>(verified_scene.is_attached("box", still_attached)) &&
                    !still_attached,
                "actual loss did not detach planning representation");
            released = observer.observe("box", Manipulator::Left);
            require(
                released.valid && std::abs(
                                      released.object_pose.pose.position.z -
                                      initial.object_pose.pose.position.z) < .01,
                "lost object physical world pose on support");
            std::cout << "ROBOT_GRASP_LOST_PASS actual release and world scene restore"
                      << std::endl;
            auto owner = std::make_unique<Robot>(node);
            require(owner->initialize(30s), "second facade readiness on same hardware");
            executing = false;
            auto abandoned = owner->start(std::make_unique<PickTask>("box"));
            const auto owner_deadline = std::chrono::steady_clock::now() + 10s;
            while (!executing && !abandoned.result() &&
                   std::chrono::steady_clock::now() < owner_deadline)
                std::this_thread::sleep_for(10ms);
            require(executing, "owner destruction fixture has an actual active trajectory");
            const auto destruction_begin = std::chrono::steady_clock::now();
            owner.reset();
            require(
                std::chrono::steady_clock::now() - destruction_begin < 10s,
                "finite active owner destruction");
            const auto abandoned_result = abandoned.wait();
            require(
                abandoned_result.state == TaskState::Canceled && abandoned.result(),
                "active owner's exact cancel and retained terminal: " + abandoned_result.message);
            retained = picked;
            std::cout
                << "ROBOT_OWNER_DESTRUCTION_PASS actual active motion canceled, Handle retained"
                << std::endl;
            std::cout << "ROBOT_PHYSICAL_PLACE_PASS cancellation, deadline, unknown/Error "
                         "recovery, Sequence stop"
                      << std::endl;
        }
        require(
            retained.result() && static_cast<bool>(retained.wait()),
            "successful Handle retained after owner destruction");
        std::cout << "ROBOT_RUNTIME_PASS one hardware, full readiness, public physical Task loop"
                  << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "ROBOT_RUNTIME_FAIL " << e.what() << std::endl;
        code = 1;
    }
    executor.cancel();
    if (spin.joinable()) spin.join();
    executor.remove_node(node);
    status.reset();
    pose_subscribers.clear();
    pose_publishers.clear();
    node.reset();
    rclcpp::shutdown();
    return code;
}
