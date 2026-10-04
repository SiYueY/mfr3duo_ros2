#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>

#include "tmr_controller.hpp"
#include "hardware_interface/handle.hpp"

namespace {
thread_local bool count_allocations = false;
thread_local std::size_t allocations = 0;
bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}
}  // namespace
void* operator new(std::size_t size) {
    if (count_allocations) ++allocations;
    if (auto* p = std::malloc(size ? size : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    using controller_interface::CallbackReturn;
    using controller_interface::return_type;
    const rclcpp_lifecycle::State lifecycle_state;
    mfr3duo_control::TmrController controller;
    if (!check(controller.init("tmr_unit") == return_type::OK, "init failed")) return 1;
    controller.get_node()->set_parameter(rclcpp::Parameter("command_timeout", .05));
    if (!check(
            controller.on_configure(lifecycle_state) == CallbackReturn::SUCCESS,
            "configure failed"))
        return 1;
    std::array<double, 4> commands{99, 99, 99, 99};
    std::array<double, 6> states{.2, 0, 0, -.1, 0, 0};
    std::vector<hardware_interface::CommandInterface> command_handles;
    std::vector<hardware_interface::StateInterface> state_handles;
    const auto command_names = controller.command_interface_configuration().names;
    const auto state_names = controller.state_interface_configuration().names;
    // Loan in reverse order to verify name-based interface mapping.
    for (int i = 3; i >= 0; --i) {
        const auto split = command_names[i].find('/');
        command_handles.emplace_back(
            command_names[i].substr(0, split), command_names[i].substr(split + 1), &commands[i]);
    }
    for (int i = 5; i >= 0; --i) {
        const auto split = state_names[i].find('/');
        state_handles.emplace_back(
            state_names[i].substr(0, split), state_names[i].substr(split + 1), &states[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> loan_commands;
    std::vector<hardware_interface::LoanedStateInterface> loan_states;
    for (auto& h : command_handles) loan_commands.emplace_back(h);
    for (auto& h : state_handles) loan_states.emplace_back(h);
    controller.assign_interfaces(std::move(loan_commands), std::move(loan_states));
    if (!check(
            controller.on_activate(lifecycle_state) == CallbackReturn::SUCCESS,
            "activate failed") ||
        !check(
            commands[0] == .2 && commands[2] == -.1 && commands[1] == 0 && commands[3] == 0,
            "activation stale commands"))
        return 1;
    auto node = rclcpp::Node::make_shared("tmr_unit_sender");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/tmr_unit/cmd_vel", 1);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(controller.get_node()->get_node_base_interface());
    const auto dt = rclcpp::Duration::from_seconds(.002);
    const auto tick = [&] {
        count_allocations = true;
        const auto result = controller.update(rclcpp::Time(0), dt);
        count_allocations = false;
        return result;
    };
    geometry_msgs::msg::Twist message;
    message.linear.y = .1;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    bool moved = false;
    while (std::chrono::steady_clock::now() < deadline) {
        publisher->publish(message);
        executor.spin_some();
        if (tick() != return_type::OK) return 1;
        if (commands[0] > .2) {
            moved = true;
            break;
        }
    }
    if (!check(moved, "callback did not command steering") ||
        !check(commands[0] - .2 <= 1.5 * .002 + 1e-12, "steering rate exceeded") ||
        !check(commands[1] == 0 && commands[3] == 0, "wheel moved before alignment"))
        return 1;
    // Simulate aligned steering, verify drive and acceleration before timeout.
    states[0] = mfr3duo_control::kPi / 2;
    states[3] = mfr3duo_control::kPi / 2;
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline && commands[1] == 0) {
        publisher->publish(message);
        executor.spin_some();
        tick();
    }
    if (!check(
            commands[1] > 0 && commands[1] <= .5 * .002 * 2 / .05 + 1e-12,
            "drive/acceleration limit"))
        return 1;
    // Invalid inputs must not refresh the monotonic watchdog, even if ROS time is fixed.
    message.linear.y = std::numeric_limits<double>::quiet_NaN();
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline && commands[1] != 0) {
        publisher->publish(message);
        executor.spin_some();
        tick();
    }
    if (!check(commands[1] == 0 && commands[3] == 0, "watchdog failed") ||
        !check(
            commands[0] == states[0] && commands[2] == states[3],
            "timeout not holding measured steering"))
        return 1;
    controller.on_deactivate(lifecycle_state);
    commands[1] = 9;
    if (!check(
            controller.on_activate(lifecycle_state) == CallbackReturn::SUCCESS,
            "reactivate failed"))
        return 1;
    tick();
    if (!check(commands[1] == 0 && commands[3] == 0, "reactivate replayed stale command")) return 1;
    // Exercise ROS-stamped odometry, TF and diagnostics publication in the
    // allocation-counted update thread, including repeated publish periods.
    for (std::int64_t i = 0; i < 10000; ++i) {
        count_allocations = true;
        const auto result = controller.update(rclcpp::Time(1000000000LL + i * 2000000LL), dt);
        count_allocations = false;
        if (!check(result == return_type::OK, "stamped odometry update failed")) return 1;
    }
    // Crossing pi must be encoded inside the backend's command limits while
    // preserving a short physical rotation, rather than clipping at pi.
    controller.on_deactivate(lifecycle_state);
    states[0] = mfr3duo_control::kPi - .001;
    states[3] = states[0];
    controller.on_activate(lifecycle_state);
    message.linear.x = -.1;
    message.linear.y = -.01;
    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline && commands[0] >= 0) {
        publisher->publish(message);
        executor.spin_some();
        tick();
    }
    if (!check(
            commands[0] < 0 && std::abs(commands[0]) <= mfr3duo_control::kPi,
            "continuous target not encoded within backend limits") ||
        !check(
            std::abs(mfr3duo_control::TmrKinematics::shortest_angle(commands[0] - states[0])) <=
                1.5 * .002 + 1e-12,
            "pi crossing violated shortest rotation/rate limit"))
        return 1;
    states[0] = .6;
    states[3] = -.8;
    controller.on_deactivate(lifecycle_state);
    if (!check(
            commands[0] == .6 && commands[2] == -.8 && commands[1] == 0 && commands[3] == 0,
            "deactivation did not hold measured steering"))
        return 1;
    controller.on_activate(lifecycle_state);
    states[0] = std::numeric_limits<double>::infinity();
    if (!check(tick() == return_type::ERROR && commands[1] == 0, "invalid state not stopped"))
        return 1;
    if (!check(allocations == 0, "update allocated memory")) return 1;
    controller.on_cleanup(lifecycle_state);
    controller.get_node()->set_parameter(rclcpp::Parameter("wheel_radius", 0.0));
    if (!check(
            controller.on_configure(lifecycle_state) == CallbackReturn::ERROR,
            "invalid configuration accepted"))
        return 1;
    rclcpp::shutdown();
    std::cout
        << "TMR controller: callback/limits/watchdog/lifecycle/invalid/update allocations passed\n";
}
