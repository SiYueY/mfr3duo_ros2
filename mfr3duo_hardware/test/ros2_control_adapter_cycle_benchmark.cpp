// Measures the ros2_control adapter cycle to confirm it adds no meaningful cost
// on top of the RobotHardware control path.
//
// Like every executable that links MuJoCo before ROS 2 middleware, this one
// must run with LD_PRELOAD pointing at the system tinyxml2; CMAKE registers it
// as a test with that environment so the requirement cannot be forgotten.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>
#include <string>
#include <vector>

#include "hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_adapter.hpp"

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

int main(int argc, char** argv) {
    using Clock = std::chrono::steady_clock;
    using hardware_interface::CallbackReturn;
    using hardware_interface::return_type;
    // Parse before rclcpp::init so the ROS argument filter cannot consume it.
    // One control period per process keeps this reliable on software OpenGL.
    const std::string selection = argc > 1 ? argv[1] : "";
    rclcpp::init(argc, argv);
    const rclcpp_lifecycle::State lifecycle_state;
    const rclcpp::Time now;
    constexpr std::size_t kCycles = 1000;
    constexpr std::array<const char*, 2> kLabels{"1000 Hz", "500 Hz"};
    constexpr std::array<const char*, 2> kPeriods{"0.001", "0.002"};

    std::vector<std::size_t> selection_indices;
    if (selection.empty()) {
        selection_indices = {0, 1};
    } else if (selection == kPeriods[0]) {
        selection_indices = {0};
    } else if (selection == kPeriods[1]) {
        selection_indices = {1};
    } else {
        std::cerr << "usage: " << argv[0] << " [0.001|0.002]   (control period in s)\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    for (const std::size_t index : selection_indices) {
        mfr3duo_hardware::Ros2ControlAdapter adapter;
        if (adapter.on_init(mfr3duo_hardware_test::make_info(kPeriods[index])) !=
                CallbackReturn::SUCCESS ||
            adapter.on_configure(lifecycle_state) != CallbackReturn::SUCCESS ||
            adapter.on_activate(lifecycle_state) != CallbackReturn::SUCCESS) {
            std::cerr << "adapter lifecycle failed at " << kLabels[index] << '\n';
            return EXIT_FAILURE;
        }
        const auto period = rclcpp::Duration::from_seconds(std::stod(kPeriods[index]));
        for (std::size_t warm = 0; warm < kCycles; ++warm) {
            if (adapter.read(now, period) != return_type::OK ||
                adapter.write(now, period) != return_type::OK) {
                return EXIT_FAILURE;
            }
        }

        std::array<double, kCycles> durations{};
        const auto before_allocations = allocations.load(std::memory_order_relaxed);
        const auto before_thread_allocations = thread_allocations;
        audit_thread = true;
        const auto total_start = Clock::now();
        for (std::size_t cycle = 0; cycle < kCycles; ++cycle) {
            const auto start = Clock::now();
            if (adapter.read(now, period) != return_type::OK ||
                adapter.write(now, period) != return_type::OK) {
                return EXIT_FAILURE;
            }
            durations[cycle] =
                std::chrono::duration<double, std::micro>(Clock::now() - start).count();
        }
        const auto total_elapsed =
            std::chrono::duration<double>(Clock::now() - total_start).count();
        audit_thread = false;
        const auto all_allocations =
            allocations.load(std::memory_order_relaxed) - before_allocations;
        const auto control_allocations = thread_allocations - before_thread_allocations;
        std::sort(durations.begin(), durations.end());
        const double budget_us = std::stod(kPeriods[index]) * 1.0e6;
        const auto overruns = std::count_if(
            durations.begin(), durations.end(),
            [budget_us](double value) { return value > budget_us; });
        std::cout << kLabels[index] << "  mean_us=" << std::fixed << std::setprecision(1)
                  << total_elapsed * 1.0e6 / kCycles << "  p99_us=" << durations[990]
                  << "  max_us=" << durations.back() << "  overruns=" << overruns
                  << "  control_thread_cpp_new_calls=" << control_allocations
                  << "  all_threads_cpp_new_calls=" << all_allocations << '\n';
        if (control_allocations != 0) {
            std::cerr << kLabels[index] << " control thread allocated " << control_allocations
                      << " times\n";
            return EXIT_FAILURE;
        }
        if (adapter.on_deactivate(lifecycle_state) != CallbackReturn::SUCCESS ||
            adapter.on_cleanup(lifecycle_state) != CallbackReturn::SUCCESS) {
            return EXIT_FAILURE;
        }
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
