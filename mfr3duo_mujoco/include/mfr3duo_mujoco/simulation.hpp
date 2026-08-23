#pragma once
#include "mfr3duo_mujoco/config.hpp"
#include "mfr3duo_mujoco/data/camera.hpp"
#include "mfr3duo_mujoco/data/robot.hpp"
#include "mfr3duo_mujoco/visibility_control.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace mfr3duo_mujoco {
struct ContactState {
    std::string geom1;
    std::string geom2;
    double distance{0.0};
};

using ContactStates = std::vector<ContactState>;
enum class SimulationStatus : std::uint8_t {
    Uninitialized,
    Stopped,
    Running,
    Paused,
    Stopping,
    Error
};

class MFR3DUO_MUJOCO_PUBLIC Simulation {
public:
    Simulation();
    ~Simulation();
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) = delete;
    Simulation& operator=(Simulation&&) = delete;
    bool initialize(const SimulationConfig& config);
    bool shutdown();
    bool start();
    bool stop();
    bool pause();
    bool resume();
    bool reset();
    bool reset(const std::string& keyframe);
    bool step(std::size_t count = 1);
    bool write_command(const RobotCommand& command);
    bool read_state(RobotState& state) const;
    std::shared_ptr<const CameraFrame> read_camera(CameraId id) const;
    bool read_contacts(ContactStates& contacts) const;
    SimulationStatus status() const;
    double time() const;
    std::uint64_t step_count() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mfr3duo_mujoco
