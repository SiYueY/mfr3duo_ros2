#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>
#include <thread>

#include "gripper_controller.hpp"
#include "hardware_interface/handle.hpp"

namespace {
using namespace std::chrono_literals;
thread_local bool audit = false;
thread_local std::size_t allocations = 0;
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
using Action = control_msgs::action::GripperCommand;
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;
struct Fixture {
    mfr3duo_control::Mfr3DuoGripperController controller;
    std::array<double, 3> commands{99, 99, 99};
    std::array<double, 4> states{.02, 0, 0, 0};
    std::vector<hardware_interface::CommandInterface> command_handles;
    std::vector<hardware_interface::StateInterface> state_handles;
    rclcpp::Node::SharedPtr node{rclcpp::Node::make_shared("gripper_unit_client")};
    rclcpp_action::Client<Action>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp_lifecycle::State lifecycle;
    Fixture(const std::string& name, bool stalling) {
        require(controller.init(name) == Return::OK, "controller init failed");
        controller.get_node()->set_parameters(
            {{"stall_timeout", .02},
             {"allow_stalling", stalling},
             {"gpio", name.find("right") != std::string::npos ? "right_gripper" : "left_gripper"}});
        require(controller.on_configure(lifecycle) == CallbackReturn::SUCCESS, "configure failed");
        const auto command_names = controller.command_interface_configuration().names;
        const auto state_names = controller.state_interface_configuration().names;
        for (int i = 2; i >= 0; --i) {
            const auto split = command_names[i].find('/');
            command_handles.emplace_back(
                command_names[i].substr(0, split), command_names[i].substr(split + 1),
                &commands[i]);
        }
        for (int i = 3; i >= 0; --i) {
            const auto split = state_names[i].find('/');
            state_handles.emplace_back(
                state_names[i].substr(0, split), state_names[i].substr(split + 1), &states[i]);
        }
        std::vector<hardware_interface::LoanedCommandInterface> loan_commands;
        std::vector<hardware_interface::LoanedStateInterface> loan_states;
        for (auto& handle : command_handles) loan_commands.emplace_back(handle);
        for (auto& handle : state_handles) loan_states.emplace_back(handle);
        controller.assign_interfaces(std::move(loan_commands), std::move(loan_states));
        require(controller.on_activate(lifecycle) == CallbackReturn::SUCCESS, "activate failed");
        require(
            commands == std::array<double, 3>{.02, .05, 10.0},
            "activation did not hold measured width");
        executor.add_node(node);
        executor.add_node(controller.get_node()->get_node_base_interface());
        client = rclcpp_action::create_client<Action>(node, "/" + name + "/gripper_cmd");
        require(client->wait_for_action_server(2s), "action server not discovered");
    }
    ~Fixture() { controller.on_cleanup(lifecycle); }
    void tick() {
        audit = true;
        const auto result = controller.update(rclcpp::Time{}, rclcpp::Duration::from_seconds(.002));
        audit = false;
        require(result == Return::OK, "update failed");
        require(allocations == 0, "update allocated");
    }
    template <typename Future>
    auto wait(Future future) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            tick();
            if (future.wait_for(0ms) == std::future_status::ready) return future.get();
        }
        throw std::runtime_error("action future deadline expired");
    }
    auto goal(double position, double effort) {
        Action::Goal goal;
        goal.command.position = position;
        goal.command.max_effort = effort;
        return wait(client->async_send_goal(goal));
    }
};
}  // namespace
void* operator new(std::size_t size) {
    if (audit) ++allocations;
    if (void* pointer = std::malloc(size ? size : 1)) return pointer;
    throw std::bad_alloc();
}
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        Fixture fixture("gripper_stall_success", true);
        auto handle = fixture.goal(.04, 0.0);
        require(
            handle && fixture.commands == std::array<double, 3>{.08, .05, 20.0},
            "wire default or finger-to-width conversion wrong");
        fixture.states[0] = .08;
        auto result = fixture.wait(fixture.client->async_get_result(handle));
        require(
            result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->reached_goal &&
                !result.result->stalled,
            "reached result invalid");
        handle = fixture.goal(.0, 7.5);
        require(handle && fixture.commands[2] == 7.5, "positive effort discarded");
        fixture.states[2] = 1.5;  // Compliance can block below the requested cap.
        result = fixture.wait(fixture.client->async_get_result(handle));
        require(
            result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->stalled &&
                !result.result->reached_goal,
            "sub-cap blocked grasp did not terminate with stalled semantics");
        fixture.states = {.04, 0, 1.0, 0.0};
        handle = fixture.goal(0.0, 7.5);
        auto ramping_result = fixture.client->async_get_result(handle);
        for (int step = 0; step < 12; ++step) {
            fixture.states[2] = 1.0 + .25 * step;
            fixture.tick();
            fixture.executor.spin_some();
            require(
                ramping_result.wait_for(0ms) != std::future_status::ready,
                "preload ramp falsely reported settled stall");
            std::this_thread::sleep_for(5ms);
        }
        result = fixture.wait(ramping_result);
        require(
            result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->stalled,
            "settled sub-cap force did not report stall");
        fixture.states = {.06, -.01, 3.0, 0.0};
        handle = fixture.goal(0.0, 0.0);
        require(bool(handle), "cancel goal rejected");
        auto canceled = fixture.wait(fixture.client->async_cancel_goal(handle));
        require(!canceled->goals_canceling.empty(), "cancel not acknowledged");
        require(
            fixture.commands == std::array<double, 3>{.06, .05, 10.0},
            "cancel did not hold snapshot with positive effort");
        result = fixture.wait(fixture.client->async_get_result(handle));
        require(
            result.code == rclcpp_action::ResultCode::CANCELED && fixture.commands[0] == .06,
            "cancel terminal or hold wrong");
        for (const double effort :
             {-1.0, 101.0, std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::infinity()})
            require(!fixture.goal(.02, effort), "invalid wire effort accepted");
        for (const double position : {-.001, .041, std::numeric_limits<double>::quiet_NaN()})
            require(!fixture.goal(position, 0.0), "invalid position accepted");
        fixture.states[0] = .04;
        require(
            fixture.controller.on_deactivate(fixture.lifecycle) == CallbackReturn::SUCCESS,
            "deactivate failed");
        require(
            fixture.commands[0] == .04 && fixture.commands[2] == 10.0,
            "deactivate retained target");
        require(
            fixture.controller.on_activate(fixture.lifecycle) == CallbackReturn::SUCCESS,
            "reactivation failed");
        fixture.tick();
        require(fixture.commands[0] == .04, "reactivation replayed goal");

        Fixture failure("gripper_stall_failure", false);
        failure.states[2] = 20.0;
        auto blocked = failure.goal(.04, 0.0);
        auto failed = failure.wait(failure.client->async_get_result(blocked));
        require(
            failed.code == rclcpp_action::ResultCode::ABORTED && failed.result->stalled &&
                !failed.result->reached_goal,
            "stall failure semantics wrong");
        // Independent clients exercise both new wire types and cross-entry ownership.
        for (const auto* side : {"left", "right"}) {
            Fixture f(std::string("official_") + side, true);
            using Move = mfr3duo_msgs::action::Move;
            using Grasp = mfr3duo_msgs::action::Grasp;
            auto move = rclcpp_action::create_client<Move>(
                f.node, std::string("/official_") + side + "/move");
            auto grasp = rclcpp_action::create_client<Grasp>(
                f.node, std::string("/official_") + side + "/grasp");
            require(
                move->wait_for_action_server(2s) && grasp->wait_for_action_server(2s),
                "new action discovery");
            Move::Goal m;
            m.width = .08;
            m.speed = .1;
            auto moving = f.wait(move->async_send_goal(m));
            require(bool(moving), "Move rejected");
            require(f.commands[0] == .08 && f.commands[1] == .1, "Move units/conversion");
            Grasp::Goal g;
            g.width = .04;
            g.speed = .05;
            g.force = 20;
            require(g.epsilon.inner == .005 && g.epsilon.outer == .005, "IDL epsilon defaults");
            require(
                !f.wait(grasp->async_send_goal(g)) && !f.goal(.02, 0), "shared busy lease missing");
            f.states[0] = .08;
            auto moved = f.wait(move->async_get_result(moving));
            require(
                moved.code == rclcpp_action::ResultCode::SUCCEEDED && moved.result->success,
                "Move result");
            f.states = {.06, 0, 2, 1};
            moving = f.wait(move->async_send_goal(m));
            auto blocked_move = f.wait(move->async_get_result(moving));
            require(
                blocked_move.code == rclcpp_action::ResultCode::ABORTED &&
                    !blocked_move.result->success,
                "blocked Move succeeded");
            auto grip = f.wait(grasp->async_send_goal(g));
            f.states = {.04, 0, 2, 1};
            auto held = f.wait(grasp->async_get_result(grip));
            require(
                held.code == rclcpp_action::ResultCode::SUCCEEDED && held.result->success,
                "in-tolerance Grasp failed");
            for (double width : {.02, .06, 0.0}) {
                grip = f.wait(grasp->async_send_goal(g));
                f.states = {width, 0, width == 0 ? 0.0 : 2.0, 1};
                auto wrong = f.wait(grasp->async_get_result(grip));
                require(
                    wrong.code == rclcpp_action::ResultCode::ABORTED && !wrong.result->success,
                    "wrong width or empty grasp succeeded");
            }
            f.states = {.08, -.02, 0, 0};
            grip = f.wait(grasp->async_send_goal(g));
            auto cancellation = f.wait(grasp->async_cancel_goal(grip));
            require(!cancellation->goals_canceling.empty(), "Grasp cancel rejected");
            auto canceled = f.wait(grasp->async_get_result(grip));
            require(
                canceled.code == rclcpp_action::ResultCode::CANCELED && !canceled.result->success,
                "Grasp cancel result");
            m.speed = 0;
            require(!f.wait(move->async_send_goal(m)), "zero speed accepted");
            g.epsilon.inner = -.1;
            require(!f.wait(grasp->async_send_goal(g)), "negative epsilon accepted");
            g.epsilon.inner = .005;
            g.force = 0;
            require(!f.wait(grasp->async_send_goal(g)), "zero force accepted");
            g.force = 20;
            f.states = {.08, -.02, 0, 0};
            grip = f.wait(grasp->async_send_goal(g));
            f.controller.on_deactivate(f.lifecycle);
            require(
                f.wait(grasp->async_get_result(grip)).code == rclcpp_action::ResultCode::ABORTED,
                "deactivation not terminal");
        }
        // Device-side timeout also terminates when no settled contact can be measured.
        Fixture timed("gripper_timeout", true);
        timed.controller.on_cleanup(timed.lifecycle);
        timed.controller.get_node()->set_parameter({"goal_timeout", .02});
        require(
            timed.controller.on_configure(timed.lifecycle) == CallbackReturn::SUCCESS,
            "timeout configure");
        require(
            timed.controller.on_activate(timed.lifecycle) == CallbackReturn::SUCCESS,
            "timeout activate");
        timed.states = {.06, -.02, 0, 0};
        auto pending = timed.goal(0, 0);
        require(
            timed.wait(timed.client->async_get_result(pending)).code ==
                rclcpp_action::ResultCode::ABORTED,
            "device timeout not terminal");
        mfr3duo_control::Mfr3DuoGripperController invalid;
        require(invalid.init("gripper_invalid") == Return::OK, "invalid fixture init failed");
        invalid.get_node()->set_parameter({"default_velocity", 0.0});
        require(
            invalid.on_configure(failure.lifecycle) == CallbackReturn::ERROR,
            "zero velocity configured");
        std::cout
            << "Gripper controller: conversion, default/positive effort, stall success/failure, "
               "cancel hold, invalid goals, lifecycle and zero update allocation PASS\n";
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
