#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>

#include "mfr3duo_hardware/mujoco_system.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_info.hpp"

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
  using hardware_interface::CallbackReturn;
  using hardware_interface::return_type;
  rclcpp::init(argc, argv);
  const rclcpp_lifecycle::State lifecycle_state;
  const rclcpp::Time now;
  constexpr std::size_t kCycles = 1000;

  for (const std::size_t steps : {1U, 2U}) {
    mfr3duo_hardware::MujocoSystem system;
    if (system.on_init(mfr3duo_hardware_test::make_info(steps)) != CallbackReturn::SUCCESS ||
        system.on_configure(lifecycle_state) != CallbackReturn::SUCCESS ||
        system.on_activate(lifecycle_state) != CallbackReturn::SUCCESS) return EXIT_FAILURE;
    const auto period = rclcpp::Duration::from_seconds(steps * 0.001);
    for (std::size_t warm = 0; warm < kCycles; ++warm)
      if (system.read(now, period) != return_type::OK ||
          system.write(now, period) != return_type::OK) return EXIT_FAILURE;

    std::array<double, kCycles> durations{};
    const auto before_allocations = allocations.load(std::memory_order_relaxed);
    const auto before_thread_allocations = thread_allocations;
    audit_thread = true;
    const auto total_start = Clock::now();
    for (std::size_t index = 0; index < kCycles; ++index) {
      const auto start = Clock::now();
      if (system.read(now, period) != return_type::OK ||
          system.write(now, period) != return_type::OK) return EXIT_FAILURE;
      durations[index] = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
    }
    const auto total_elapsed =
        std::chrono::duration<double>(Clock::now() - total_start).count();
    audit_thread = false;
    const auto all_allocations = allocations.load(std::memory_order_relaxed) - before_allocations;
    const auto control_allocations = thread_allocations - before_thread_allocations;
    std::sort(durations.begin(), durations.end());
    const double budget_us = steps * 1000.0;
    const auto overruns = std::count_if(durations.begin(), durations.end(),
                                        [budget_us](double value) { return value > budget_us; });
    std::cout << (steps == 1 ? "1000 Hz" : "500 Hz")
              << "  mean_us=" << std::fixed << std::setprecision(1)
              << total_elapsed * 1.0e6 / kCycles
              << "  p99_us=" << durations[990]
              << "  max_us=" << durations.back()
              << "  overruns=" << overruns
              << "  control_thread_cpp_new_calls=" << control_allocations
              << "  all_threads_cpp_new_calls=" << all_allocations << '\n';
    if (system.on_deactivate(lifecycle_state) != CallbackReturn::SUCCESS ||
        system.on_cleanup(lifecycle_state) != CallbackReturn::SUCCESS) return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
