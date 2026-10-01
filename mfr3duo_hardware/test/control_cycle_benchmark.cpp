// Benchmarks the RobotHardware control path directly, without ROS.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>
#include <string_view>

#include "mfr3duo_hardware/robot_hardware.hpp"

namespace {

std::atomic<std::size_t> allocations{0};
thread_local bool audit_thread{false};
thread_local std::size_t thread_allocations{0};

}  // namespace

void* operator new(std::size_t size) {
    allocations.fetch_add(1, std::memory_order_relaxed);
    if (audit_thread) ++thread_allocations;
    if (void* memory = std::malloc(size)) return memory;
    throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
    allocations.fetch_add(1, std::memory_order_relaxed);
    if (audit_thread) ++thread_allocations;
    if (void* memory = std::malloc(size)) return memory;
    throw std::bad_alloc();
}

void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }

namespace {

mfr3duo_hardware::RobotCommand hold_command(const mfr3duo_hardware::RobotState& state) {
    mfr3duo_hardware::RobotCommand command;
    command.left_arm.mode = mfr3duo_hardware::JointControlMode::Position;
    command.right_arm.mode = mfr3duo_hardware::JointControlMode::Position;
    for (std::size_t index = 0; index < mfr3duo_hardware::kArmJointCount; ++index) {
        command.left_arm.joints[index].position = state.left_arm.joints[index].position;
        command.right_arm.joints[index].position = state.right_arm.joints[index].position;
    }
    command.spine.position = state.spine.position;
    command.tmr.front_steering_position = state.tmr.front_steering.position;
    command.tmr.rear_steering_position = state.tmr.rear_steering.position;
    command.left_gripper.width = state.left_gripper.width;
    command.right_gripper.width = state.right_gripper.width;
    return command;
}

int run_benchmark(std::chrono::nanoseconds control_period, const char* label) {
    using Clock = std::chrono::steady_clock;
    constexpr std::size_t kCycles = 1000;

    mfr3duo_hardware::RobotHardware robot;
    mfr3duo_hardware::RobotHardwareOptions options;
    options.control_period = control_period;
    if (!robot.initialize(options) || !robot.activate()) return EXIT_FAILURE;

    mfr3duo_hardware::RobotState state;
    if (!robot.read_state(state)) return EXIT_FAILURE;
    mfr3duo_hardware::ImuState imu;
    if (!robot.read_state(imu)) return EXIT_FAILURE;
    const auto command = hold_command(state);

    for (int warm = 0; warm < 1000; ++warm) {
        if (!robot.write_command(command) || !robot.update() || !robot.read_state(state)) {
            return EXIT_FAILURE;
        }
    }

    // The control path must not allocate on the calling thread at all.
    constexpr int kCalls = 1000;
    audit_thread = true;
    thread_allocations = 0;
    for (int index = 0; index < kCalls; ++index) {
        if (!robot.write_command(command)) return EXIT_FAILURE;
    }
    const auto write_allocations = thread_allocations;
    thread_allocations = 0;
    for (int index = 0; index < kCalls; ++index) {
        if (!robot.read_state(state)) return EXIT_FAILURE;
    }
    const auto read_allocations = thread_allocations;
    thread_allocations = 0;
    for (int index = 0; index < kCalls; ++index) {
        if (!robot.read_state(imu)) return EXIT_FAILURE;
    }
    const auto imu_allocations = thread_allocations;
    audit_thread = false;
    std::cout << label << "  write_cpp_new_per_call=" << write_allocations / 1000.0
              << "  state_read_cpp_new_per_call=" << read_allocations / 1000.0
              << "  imu_read_cpp_new_per_call=" << imu_allocations / 1000.0 << '\n';
    if (write_allocations != 0 || read_allocations != 0 || imu_allocations != 0) {
        std::cerr << label << " control path allocated on the calling thread\n";
        return EXIT_FAILURE;
    }

    std::array<double, kCycles> durations{};
    std::array<double, kCycles> cycle_durations{};
    const auto before_allocations = allocations.load(std::memory_order_relaxed);
    const auto before_thread_allocations = thread_allocations;
    audit_thread = true;
    const auto total_start = Clock::now();
    for (std::size_t index = 0; index < kCycles; ++index) {
        const auto begin = Clock::now();
        if (!robot.write_command(command)) return EXIT_FAILURE;
        const auto cycle_begin = Clock::now();
        if (!robot.update() || !robot.read_state(state)) return EXIT_FAILURE;
        const auto cycle_end = Clock::now();
        durations[index] = std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
        cycle_durations[index] =
            std::chrono::duration<double, std::micro>(cycle_end - cycle_begin).count();
    }
    const auto total_elapsed = std::chrono::duration<double>(Clock::now() - total_start).count();
    const auto allocations_used = allocations.load(std::memory_order_relaxed) - before_allocations;
    audit_thread = false;
    const auto thread_allocations_used = thread_allocations - before_thread_allocations;

    std::sort(durations.begin(), durations.end());
    std::sort(cycle_durations.begin(), cycle_durations.end());
    const double budget_us = std::chrono::duration<double, std::micro>(control_period).count();
    const auto overruns = std::count_if(
        durations.begin(), durations.end(),
        [budget_us](double value) { return value > budget_us; });
    std::cout << label << "  mean_us=" << std::fixed << std::setprecision(1)
              << total_elapsed * 1.0e6 / kCycles << "  p99_us=" << durations[990]
              << "  max_us=" << durations.back() << "  update_p99_us=" << cycle_durations[990]
              << "  update_max_us=" << cycle_durations.back() << "  overruns=" << overruns
              << "  cpp_new_calls=" << allocations_used
              << "  control_thread_cpp_new_calls=" << thread_allocations_used << '\n';
    if (thread_allocations_used != 0) {
        std::cerr << label << " control cycle allocated on the control thread\n";
        return EXIT_FAILURE;
    }

    if (!robot.deactivate() || !robot.shutdown()) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}

}  // namespace

// Creating a second simulation in the same process is unreliable on software
// OpenGL (the render worker can time out), so ctest runs one control period per
// process. Calling without an argument keeps the manual two-period run.
int main(int argc, char** argv) {
    const std::string_view selection = argc > 1 ? argv[1] : "";
    if (selection.empty()) {
        if (run_benchmark(std::chrono::milliseconds(1), "1000 Hz") != EXIT_SUCCESS) {
            return EXIT_FAILURE;
        }
        return run_benchmark(std::chrono::milliseconds(2), "500 Hz");
    }
    if (selection == "1") return run_benchmark(std::chrono::milliseconds(1), "1000 Hz");
    if (selection == "2") return run_benchmark(std::chrono::milliseconds(2), "500 Hz");
    std::cerr << "usage: " << argv[0] << " [1|2]   (control period in ms)\n";
    return EXIT_FAILURE;
}
