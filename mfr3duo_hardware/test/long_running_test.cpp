#include <cmath>
#include <cstdlib>
#include <iostream>

#include "mfr3duo_mujoco/simulation.hpp"

int main() {
  mfr3duo_mujoco::SimulationOptions options;
  options.viewer_enabled = false;
  options.camera_width = 320;
  options.camera_height = 180;
  options.camera_period = 0.04;
  mfr3duo_mujoco::Simulation simulation;
  if (!simulation.initialize(options)) return EXIT_FAILURE;
  mfr3duo_mujoco::RobotState before;
  if (!simulation.read_state(before)) return EXIT_FAILURE;

  constexpr std::uint64_t kStepsPerBatch = 1000;
  constexpr std::uint64_t kBatches = 60;
  std::uint64_t previous_sequence = before.sequence;
  for (std::uint64_t batch = 0; batch < kBatches; ++batch) {
    if (!simulation.step(kStepsPerBatch)) {
      std::cerr << "simulation step failed at batch " << batch << '\n';
      return EXIT_FAILURE;
    }
    mfr3duo_mujoco::RobotState state;
    mfr3duo_mujoco::LaserScan lidar;
    mfr3duo_mujoco::CameraFrame camera;
    if (!simulation.read_state(state) ||
        !simulation.read_state(mfr3duo_mujoco::Lidar::Front, lidar) ||
        !simulation.read_state(mfr3duo_mujoco::Camera::FrontColor, camera) ||
        state.sequence <= previous_sequence ||
        state.step != (batch + 1) * kStepsPerBatch ||
        !std::isfinite(state.simulation_time) ||
        lidar.ranges.empty() || camera.image.data.empty()) {
      std::cerr << "state or sensor stream invalid at batch " << batch << '\n';
      return EXIT_FAILURE;
    }
    previous_sequence = state.sequence;
  }
  return simulation.shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
