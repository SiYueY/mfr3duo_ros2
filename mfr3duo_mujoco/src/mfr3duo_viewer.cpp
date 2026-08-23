#include "runtime_config_file.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco_simulation/simulation.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> stop_requested{false};
void request_stop(int) { stop_requested.store(true); }

const char* usage() { return "Usage: mfr3duo_mujoco_viewer [--model ABSOLUTE_MJCF_PATH]"; }

enum class ArgumentsResult { Run, Help, Error };

ArgumentsResult parse_arguments(
    int argc, char** argv, std::string& model_path, std::string& error) {
    model_path.clear();
    error.clear();
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--help" || argument == "-h") return ArgumentsResult::Help;
        if (argument != "--model") {
            error = "unknown argument: " + argument;
            return ArgumentsResult::Error;
        }
        if (++index == argc) {
            error = "--model requires an absolute MJCF path";
            return ArgumentsResult::Error;
        }
        if (!model_path.empty()) {
            error = "--model may only be specified once";
            return ArgumentsResult::Error;
        }
        model_path = argv[index];
    }
    return ArgumentsResult::Run;
}
}  // namespace

int main(int argc, char** argv) {
    std::string model_path;
    std::string error;
    const auto parsed = parse_arguments(argc, argv, model_path, error);
    if (parsed == ArgumentsResult::Help) {
        std::cout << usage() << '\n';
        return 0;
    }
    if (parsed == ArgumentsResult::Error) {
        std::cerr << error << '\n' << usage() << '\n';
        return 2;
    }
    try {
        if (model_path.empty()) {
            model_path = (std::filesystem::path(
                              ament_index_cpp::get_package_share_directory("mfr3duo_description")) /
                          "mjcf" / "scene.xml")
                             .string();
        }
    } catch (const std::exception& exception) {
        std::cerr << "cannot locate mfr3duo_description: " << exception.what() << '\n';
        return 1;
    }

    mfr3duo_mujoco::RuntimeConfigFile config;
    if (!config.create(model_path, true)) {
        std::cerr << "cannot create viewer configuration: " << config.error() << '\n';
        return 1;
    }
    mujoco_simulation::Simulation simulation;
    if (!simulation.initialize(config.path().string())) {
        std::cerr << "cannot initialize MuJoCo viewer simulation\n";
        return 1;
    }
    if (!simulation.start()) {
        std::cerr << "cannot start MuJoCo viewer; verify DISPLAY and GLFW/EGL support\n";
        simulation.shutdown();
        return 1;
    }
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    while (!stop_requested.load() &&
           simulation.status() == mujoco_simulation::SimulationStatus::Running)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    simulation.stop();
    simulation.shutdown();
    return 0;
}
