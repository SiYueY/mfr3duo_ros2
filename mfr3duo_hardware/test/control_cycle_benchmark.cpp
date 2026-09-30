#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>
#include <string_view>

#include "mfr3duo_mujoco/simulation.hpp"

namespace {
std::atomic<std::size_t> allocations{0};
thread_local bool audit_thread{false};
thread_local std::size_t thread_allocations{0};
}

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

int main(int argc, char** argv) {
  using Clock = std::chrono::steady_clock;
  mfr3duo_mujoco::SimulationOptions options;
  options.viewer_enabled = false;
  const std::string_view selection = argc > 1 ? argv[1] : "";
  options.cameras_enabled = selection != "--no-sensors" && selection != "--no-camera";
  options.lidars_enabled = selection != "--no-sensors" && selection != "--no-lidar";
  options.imu_enabled = selection != "--no-sensors" && selection != "--no-imu";
  options.camera_width = 320;
  options.camera_height = 180;
  options.camera_period = 0.04;
  mfr3duo_mujoco::Simulation simulation;
  if (!simulation.initialize(options)) return EXIT_FAILURE;

  mfr3duo_mujoco::RobotState state;
  mfr3duo_mujoco::ImuState imu;
  if (!simulation.read_state(state)) return EXIT_FAILURE;
  mfr3duo_mujoco::RobotCommand command;
  for (std::size_t index = 0; index < mfr3duo_mujoco::kArmJointCount; ++index) {
    command.left_arm.joints[index].position = state.left_arm.joints[index].position;
    command.right_arm.joints[index].position = state.right_arm.joints[index].position;
  }
  command.spine.position = state.spine.position;
  command.tmr.front_steering_position = state.tmr.front_steering.position;
  command.tmr.rear_steering_position = state.tmr.rear_steering.position;
  command.left_gripper.width = state.left_gripper.width;
  command.right_gripper.width = state.right_gripper.width;

  for (int warm = 0; warm < 100; ++warm)
    if (!simulation.write_command(command) || !simulation.read_state(state)) return EXIT_FAILURE;
  const auto before_writes = allocations.load(std::memory_order_relaxed);
  for (int index = 0; index < 1000; ++index)
    if (!simulation.write_command(command)) return EXIT_FAILURE;
  const auto write_allocations = allocations.load(std::memory_order_relaxed) - before_writes;
  const auto before_reads = allocations.load(std::memory_order_relaxed);
  for (int index = 0; index < 1000; ++index)
    if (!simulation.read_state(state)) return EXIT_FAILURE;
  const auto read_allocations = allocations.load(std::memory_order_relaxed) - before_reads;
  std::cout << "camera=" << options.cameras_enabled
            << " lidar=" << options.lidars_enabled
            << " imu=" << options.imu_enabled
            << "  write_cpp_new_per_call=" << write_allocations / 1000.0
            << "  motion_read_cpp_new_per_call=" << read_allocations / 1000.0 << '\n';

  constexpr std::size_t kCycles = 1000;
  std::array<double, kCycles> durations{};
  std::array<double, kCycles> step_durations{};
  for (const auto steps : {1U, 2U}) {
    for (int warm = 0; warm < 1000; ++warm) {
      if (!simulation.write_command(command) || !simulation.step(steps) ||
          !simulation.read_state(state) ||
          (options.imu_enabled && !simulation.read_state(imu))) return EXIT_FAILURE;
    }
    const auto before_allocations = allocations.load(std::memory_order_relaxed);
    const auto before_thread_allocations = thread_allocations;
    audit_thread = true;
    const auto total_start = Clock::now();
    for (std::size_t index = 0; index < kCycles; ++index) {
      const auto begin = Clock::now();
      if (!simulation.write_command(command)) return EXIT_FAILURE;
      const auto step_begin = Clock::now();
      if (!simulation.step(steps)) return EXIT_FAILURE;
      const auto step_end = Clock::now();
      if (!simulation.read_state(state) ||
          (options.imu_enabled && !simulation.read_state(imu))) return EXIT_FAILURE;
      durations[index] = std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
      step_durations[index] =
          std::chrono::duration<double, std::micro>(step_end - step_begin).count();
    }
    const auto total_elapsed =
        std::chrono::duration<double>(Clock::now() - total_start).count();
    const auto allocations_used =
        allocations.load(std::memory_order_relaxed) - before_allocations;
    audit_thread = false;
    const auto thread_allocations_used = thread_allocations - before_thread_allocations;
    std::sort(durations.begin(), durations.end());
    std::sort(step_durations.begin(), step_durations.end());
    const double budget_us = steps * 1000.0;
    const auto overruns = std::count_if(durations.begin(), durations.end(),
                                        [budget_us](double value) { return value > budget_us; });
    std::cout << (steps == 1 ? "1000 Hz" : "500 Hz")
              << "  mean_us=" << std::fixed << std::setprecision(1)
              << total_elapsed * 1.0e6 / kCycles
              << "  p99_us=" << durations[990]
              << "  max_us=" << durations.back()
              << "  step_p99_us=" << step_durations[990]
              << "  step_max_us=" << step_durations.back()
              << "  overruns=" << overruns
              << "  cpp_new_calls=" << allocations_used << '\n';
    std::cout << "control_thread_cpp_new_calls=" << thread_allocations_used << '\n';
  }
  return simulation.shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
