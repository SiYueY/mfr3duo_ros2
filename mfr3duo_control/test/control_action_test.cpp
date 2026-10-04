#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <iostream>
#include <mutex>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include "mfr3duo_control/control.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "mfr3duo_msgs/action/move.hpp"
#include "mfr3duo_msgs/action/grasp.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace mfr3duo_control;
using Trajectory = control_msgs::action::FollowJointTrajectory;
using GripperAction = control_msgs::action::GripperCommand;
using Clock = std::chrono::steady_clock;
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

struct Server {
    enum Mode { Immediate, Reject, Hold, RejectCancel, LateAccepted, LateRejected, FailedResult };
    std::atomic<Mode> mode{Immediate};
    std::mutex mutex;
    std::condition_variable changed;
    bool entered{false}, released{false}, accepted{false}, cancel_received{false}, finish{false};
    rclcpp::Node::SharedPtr node{rclcpp::Node::make_shared("control_fake_servers")};
    rclcpp::CallbackGroup::SharedPtr group{
        node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)};
    std::vector<rclcpp_action::Server<Trajectory>::SharedPtr> trajectories;
    std::vector<rclcpp_action::Server<GripperAction>::SharedPtr> grippers;
    std::vector<rclcpp_action::Server<mfr3duo_msgs::action::Move>::SharedPtr> moves;
    std::vector<rclcpp_action::Server<mfr3duo_msgs::action::Grasp>::SharedPtr> grasps;
    std::vector<std::function<bool()>> pending;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr states;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity;
    rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::string> names;
    bool publish_states{false};

    Server() {
        for (const auto* side : {"left", "right"})
            for (int i = 1; i <= 7; ++i)
                names.push_back(std::string(side) + "_fr3v2_1_joint" + std::to_string(i));
        names.insert(
            names.end(), {"franka_spine_vertical_joint", "left_fr3v2_1_finger_joint1",
                          "right_fr3v2_1_finger_joint1"});
        states = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        description = node->create_publisher<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local());
        std_msgs::msg::String xml;
        xml.data = "<robot name='fixture'><link name='base'/>";
        for (const auto& name : names)
            xml.data += "<link name='" + name + "_link'/><joint name='" + name +
                        "' type='prismatic'>"
                        "<parent link='base'/><child link='" +
                        name +
                        "_link'/><limit lower='-3' upper='3' velocity='2' effort='100'/></joint>";
        xml.data += "</robot>";
        description->publish(xml);
        rclcpp::SubscriptionOptions options;
        options.callback_group = group;
        velocity = node->create_subscription<geometry_msgs::msg::Twist>(
            "/tmr_controller/cmd_vel", 1, [](const geometry_msgs::msg::Twist&) {}, options);
        controllers = node->create_service<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers",
            [](const controller_manager_msgs::srv::ListControllers::Request::SharedPtr,
               controller_manager_msgs::srv::ListControllers::Response::SharedPtr result) {
                for (const auto* name :
                     {"joint_state_broadcaster", "imu_broadcaster", "left_arm_controller",
                      "right_arm_controller", "spine_controller", "left_gripper_controller",
                      "right_gripper_controller", "tmr_controller"}) {
                    controller_manager_msgs::msg::ControllerState state;
                    state.name = name;
                    state.state = "active";
                    result->controller.push_back(state);
                }
            },
            rmw_qos_profile_services_default, group);
        for (const auto* name : {"left_arm_controller", "right_arm_controller", "spine_controller"})
            trajectories.push_back(
                make<Trajectory>(std::string("/") + name + "/follow_joint_trajectory"));
        for (const auto* name : {"left_gripper_controller", "right_gripper_controller"})
            grippers.push_back(make<GripperAction>(std::string("/") + name + "/gripper_cmd"));
        for (const auto* name : {"left_gripper_controller", "right_gripper_controller"}) {
            moves.push_back(make<mfr3duo_msgs::action::Move>(std::string("/") + name + "/move"));
            grasps.push_back(make<mfr3duo_msgs::action::Grasp>(std::string("/") + name + "/grasp"));
        }
        timer = node->create_wall_timer(
            5ms,
            [this] {
                std::lock_guard<std::mutex> lock(mutex);
                if (publish_states) {
                    sensor_msgs::msg::JointState message;
                    message.header.stamp = node->now();
                    message.name = names;
                    message.position.resize(names.size(), 0.0);
                    // GPIOs have no standard position; broadcaster can include NaN.
                    message.name.push_back("left_gripper");
                    message.position.push_back(std::numeric_limits<double>::quiet_NaN());
                    states->publish(message);
                }
                sensor_msgs::msg::JointState passive;
                passive.header.stamp = node->now();
                passive.name = {"rocker_arm_joint"};
                passive.position = {.01};
                states->publish(passive);
                if (finish) {
                    pending.erase(
                        std::remove_if(
                            pending.begin(), pending.end(),
                            [](const auto& done) { return done(); }),
                        pending.end());
                }
            },
            group);
    }
    template <typename Action>
    typename rclcpp_action::Server<Action>::SharedPtr make(const std::string& name) {
        using Handle = rclcpp_action::ServerGoalHandle<Action>;
        return rclcpp_action::create_server<Action>(
            node, name,
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal>) {
                const auto current = mode.load();
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    entered = true;
                    changed.notify_all();
                    if (current == LateAccepted || current == LateRejected)
                        if (!changed.wait_for(lock, 2s, [&] { return released; }))
                            return rclcpp_action::GoalResponse::REJECT;
                }
                return current == Reject || current == LateRejected
                           ? rclcpp_action::GoalResponse::REJECT
                           : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](std::shared_ptr<Handle>) {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    cancel_received = true;
                }
                changed.notify_all();
                return mode.load() == RejectCancel ? rclcpp_action::CancelResponse::REJECT
                                                   : rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](std::shared_ptr<Handle> handle) {
                const auto done = [this, handle] {
                    auto result = std::make_shared<typename Action::Result>();
                    if constexpr (std::is_same_v<Action, GripperAction>) {
                        result->position = handle->get_goal()->command.position;
                        result->reached_goal = true;
                    } else if constexpr (
                        std::is_same_v<Action, mfr3duo_msgs::action::Move> ||
                        std::is_same_v<Action, mfr3duo_msgs::action::Grasp>) {
                        result->success = mode.load() != FailedResult;
                        if (!result->success) result->error = "device rejected grasp width";
                    } else
                        result->error_code = Trajectory::Result::SUCCESSFUL;
                    if (!handle->is_active()) return true;
                    if (cancel_received && mode.load() != RejectCancel && !handle->is_canceling())
                        return false;
                    if (handle->is_canceling())
                        handle->canceled(result);
                    else
                        handle->succeed(result);
                    return true;
                };
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    accepted = true;
                    if (mode.load() == Immediate || mode.load() == FailedResult)
                        done();
                    else
                        pending.push_back(done);
                }
                changed.notify_all();
            },
            rcl_action_server_get_default_options(), group);
    }
    void reset(Mode value) {
        std::lock_guard<std::mutex> lock(mutex);
        require(pending.empty(), "fixture still owns unfinished goal");
        entered = released = accepted = cancel_received = finish = false;
        mode.store(value);
    }
    void wait(bool Server::*field) {
        std::unique_lock<std::mutex> lock(mutex);
        require(
            changed.wait_for(lock, 2s, [&] { return this->*field; }),
            "fixture event deadline expired");
    }
    void release() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            released = true;
        }
        changed.notify_all();
    }
    void complete() {
        std::lock_guard<std::mutex> lock(mutex);
        finish = true;
    }
};

int main(int argc, char** argv) {
    setenv("ROS_DOMAIN_ID", std::to_string(20 + getpid() % 70).c_str(), 1);
    rclcpp::init(argc, argv);
    Server server;
    auto node = rclcpp::Node::make_shared(
        "control_action_client", rclcpp::NodeOptions().parameter_overrides(
                                     {{"execution.goal_response_timeout", .08},
                                      {"execution.timeout_margin", .1},
                                      {"execution.cancel_timeout", .1},
                                      {"execution.terminal_timeout", .12},
                                      {"execution.gripper_timeout", .12}}));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4);
    executor.add_node(node);
    executor.add_node(server.node);
    std::thread spinner([&] { executor.spin(); });
    int status = 0;
    try {
        Control control(node);
        require(
            control.get_spine_position().result.code == ErrorCode::StateUnavailable,
            "unreceived state not distinguished");
        {
            std::lock_guard<std::mutex> lock(server.mutex);
            server.publish_states = true;
        }
        require(bool(control.initialize(3s)), "initialize failed");
        require(bool(control.initialize(100ms)), "initialize not idempotent");
        require(control.is_ready(), "not ready");
        require(
            bool(control.command_arm_joint_position(Arm::Left, std::vector<double>(7, .1), 10ms)),
            "trajectory success failed");
        require(bool(control.command_spine_position(.2, 10ms)), "spine success failed");
        require(bool(control.command_gripper(Gripper::Left, .04)), "default gripper failed");
        for (const auto gripper : {Gripper::Left, Gripper::Right}) {
            require(bool(control.move_gripper(gripper, .08, .05)), "new Move facade failed");
            require(bool(control.grasp_gripper(gripper, .04, .05, 20)), "new Grasp facade failed");
        }
        auto opening = control.get_gripper_width(Gripper::Left);
        require(
            bool(opening) && opening.value == 0, "opening state unavailable or conversion wrong");
        server.reset(Server::FailedResult);
        require(
            control.grasp_gripper(Gripper::Left, .04, .05, 20).code == ErrorCode::ExecutionFailed,
            "ROS SUCCEEDED with false device result was accepted");
        server.reset(Server::Immediate);
        require(
            control.move_gripper(Gripper::Left, .1, .05).code == ErrorCode::InvalidArgument &&
                control.grasp_gripper(Gripper::Left, .04, .05, 20, -.1).code ==
                    ErrorCode::InvalidArgument,
            "new facade parameter validation missing");
        for (double effort : {0.0, -1.0, 101.0, std::numeric_limits<double>::quiet_NaN()})
            require(
                control.command_gripper(Gripper::Left, .01, effort).code ==
                    ErrorCode::InvalidArgument,
                "invalid explicit effort accepted");
        require(
            control.command_arm_joint_position(Arm::Left, std::vector<double>(6), 1ms).code ==
                ErrorCode::InvalidArgument,
            "wrong dimension accepted");
        require(
            control.command_spine_position(std::numeric_limits<double>::infinity(), 1ms).code ==
                ErrorCode::InvalidArgument,
            "nonfinite position accepted");
        server.reset(Server::Reject);
        require(
            control.command_gripper(Gripper::Left, .04).code == ErrorCode::GoalRejected,
            "goal rejection lost");
        require(control.is_ready(), "rejection blocked resource");

        server.reset(Server::Hold);
        auto motion = std::async(std::launch::async, [&] {
            return control.command_arm_joint_position(Arm::Left, std::vector<double>(7), 2s);
        });
        server.wait(&Server::accepted);
        require(
            control.command_gripper(Gripper::Left, .04).code == ErrorCode::Busy,
            "concurrent execution not Busy");
        auto cancel = std::async(std::launch::async, [&] { return control.stop_arm(Arm::Left); });
        server.wait(&Server::cancel_received);
        require(
            cancel.wait_for(0ms) != std::future_status::ready &&
                motion.wait_for(0ms) != std::future_status::ready,
            "cancel response mistaken for termination");
        server.complete();
        require(
            cancel.get().code == ErrorCode::Canceled && motion.get().code == ErrorCode::Canceled,
            "explicit cancellation did not confirm terminal");
        require(
            control.stop_arm(Arm::Left).code == ErrorCode::Canceled,
            "already canceled result lost");

        server.reset(Server::Hold);
        auto grip_motion = std::async(
            std::launch::async, [&] { return control.grasp_gripper(Gripper::Left, .04, .05, 20); });
        server.wait(&Server::accepted);
        auto grip_cancel =
            std::async(std::launch::async, [&] { return control.stop_gripper(Gripper::Left); });
        server.wait(&Server::cancel_received);
        require(
            grip_cancel.wait_for(0ms) != std::future_status::ready &&
                grip_motion.wait_for(0ms) != std::future_status::ready,
            "new Grasp cancel ACK mistaken for termination");
        server.complete();
        require(
            grip_cancel.get().code == ErrorCode::Canceled &&
                grip_motion.get().code == ErrorCode::Canceled,
            "new Grasp did not confirm canceled result");
        server.reset(Server::RejectCancel);
        motion = std::async(std::launch::async, [&] {
            return control.command_arm_joint_position(Arm::Right, std::vector<double>(7), 2s);
        });
        server.wait(&Server::accepted);
        cancel = std::async(std::launch::async, [&] { return control.stop_arm(Arm::Right); });
        server.wait(&Server::cancel_received);
        server.complete();
        require(
            bool(cancel.get()) && bool(motion.get()),
            "already terminated / rejected cancel race failed");

        server.reset(Server::Hold);
        require(
            control.move_gripper(Gripper::Left, .08, .05).code == ErrorCode::CancelFailed,
            "unknown Move termination not CancelFailed");
        require(!control.is_ready(), "unknown termination reported ready");
        require(
            control.command_gripper(Gripper::Left, .04).code ==
                ErrorCode::PreviousOperationNotTerminated,
            "unknown resource not blocked");
        require(
            control.initialize(100ms).code == ErrorCode::PreviousOperationNotTerminated,
            "initialize erased unknown state");
        server.complete();
        const auto ready = [&] {
            const auto deadline = Clock::now() + 2s;
            std::unique_lock<std::mutex> lock(server.mutex);
            while (Clock::now() < deadline) {
                lock.unlock();
                const bool value = control.is_ready();
                lock.lock();
                if (value && server.pending.empty()) return;
                server.changed.wait_until(lock, std::min(deadline, Clock::now() + 5ms));
            }
            throw std::runtime_error("late terminal did not unblock resource");
        };
        ready();

        server.reset(Server::LateAccepted);
        motion = std::async(
            std::launch::async, [&] { return control.command_gripper(Gripper::Left, .04); });
        server.wait(&Server::entered);
        require(
            motion.get().code == ErrorCode::CancelFailed,
            "late pending response did not retain identity");
        server.release();
        server.wait(&Server::accepted);
        server.wait(&Server::cancel_received);
        server.complete();
        ready();

        server.reset(Server::LateRejected);
        motion = std::async(
            std::launch::async, [&] { return control.command_gripper(Gripper::Left, .04); });
        server.wait(&Server::entered);
        require(
            motion.get().code == ErrorCode::CancelFailed,
            "late rejection prematurely released resource");
        server.release();
        ready();
        server.reset(Server::Immediate);
        require(bool(control.command_gripper(Gripper::Left, .04)), "resource did not recover");
        require(
            bool(control.command_base_velocity({.1, .0, .0})) && bool(control.stop_base()),
            "velocity publisher failed");
        {
            std::lock_guard<std::mutex> lock(server.mutex);
            server.publish_states = false;
        }
        const auto stale_deadline = Clock::now() + 2s;
        bool stale = false;
        while (Clock::now() < stale_deadline) {
            if (control.get_spine_position().result.code == ErrorCode::StateStale) {
                stale = true;
                break;
            }
            std::unique_lock<std::mutex> lock(server.mutex);
            server.changed.wait_until(lock, std::min(stale_deadline, Clock::now() + 10ms));
        }
        require(stale && !control.is_ready(), "stale state incorrectly usable");
        require(
            control.initialize(30ms).code == ErrorCode::Timeout, "stale readiness not rechecked");
        {
            std::lock_guard<std::mutex> lock(server.mutex);
            server.publish_states = true;
        }
        require(bool(control.initialize(2s)), "state readiness retry failed");

        std::cout << "Control: result, Busy, effort validation, cancel/terminal, rejected cancel "
                     "race, late acceptance/rejection and unknown resource recovery PASS\n";
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        status = 1;
        server.release();
        server.complete();
    }
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return status;
}
