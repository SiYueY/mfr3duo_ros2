#include "mfr3duo_mujoco/simulation.hpp"
#include "model_contract.hpp"
#include "mujoco_simulation/simulation.hpp"
#include "robot_command.hpp"
#include "robot_state.hpp"
#include "runtime_config_file.hpp"
#include <iostream>
#include <utility>

namespace mfr3duo_mujoco {
class CameraFrame::Impl {
public:
    explicit Impl(std::shared_ptr<const mujoco_simulation::CameraState> state)
    : state_(std::move(state)) {
        metadata_.sequence = state_->sequence;
        metadata_.timestamp = state_->timestamp;
        metadata_.frame_id = state_->frame_id;
        metadata_.optical_frame_id = state_->optical_frame_id;
        metadata_.height = state_->camera_info.height;
        metadata_.width = state_->camera_info.width;
        metadata_.distortion_model = state_->camera_info.distortion_model;
        metadata_.distortion = state_->camera_info.d;
    }
    ImageView image(bool depth) const {
        const auto& image = depth ? state_->depth_image : state_->image;
        return {image.timestamp, image.data.data(), image.data.size(), image.height,
                image.width,     image.step,        image.encoding};
    }
    const CameraMetadata& metadata() const { return metadata_; }

private:
    std::shared_ptr<const mujoco_simulation::CameraState> state_;
    CameraMetadata metadata_;
};
CameraFrame::CameraFrame(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}
CameraFrame::~CameraFrame() = default;
const CameraMetadata& CameraFrame::metadata() const { return impl_->metadata(); }
ImageView CameraFrame::color() const { return impl_->image(false); }
ImageView CameraFrame::depth() const { return impl_->image(true); }

class Simulation::Impl {
public:
    mujoco_simulation::Simulation simulation;
    RuntimeConfigFile config;
};
Simulation::Simulation() : impl_(std::make_unique<Impl>()) {}
Simulation::~Simulation() { shutdown(); }

bool Simulation::initialize(const SimulationConfig& config) {
    impl_->simulation.shutdown();
    if (!impl_->config.create(config.model_path, false)) return false;
    if (impl_->simulation.initialize(impl_->config.path().string())) return true;
    impl_->config.reset();
    return false;
}
bool Simulation::shutdown() {
    const bool result = impl_->simulation.shutdown();
    impl_->config.reset();
    return result;
}
bool Simulation::start() { return impl_->simulation.start(); }
bool Simulation::stop() { return impl_->simulation.stop(); }
bool Simulation::pause() { return impl_->simulation.pause(); }
bool Simulation::resume() { return impl_->simulation.resume(); }
bool Simulation::reset() { return impl_->simulation.reset(); }
bool Simulation::reset(const std::string& keyframe) { return impl_->simulation.reset(keyframe); }
bool Simulation::step(std::size_t count) { return impl_->simulation.step(count); }
bool Simulation::write_command(const RobotCommand& command) {
    mujoco_simulation::RobotCommand lower;
    return to_runtime_command(command, lower) && impl_->simulation.write_command(lower);
}
bool Simulation::read_state(RobotState& state) const {
    std::shared_ptr<const mujoco_simulation::RobotState> lower;
    return impl_->simulation.read_state(lower) && lower && from_runtime_state(*lower, state);
}
std::shared_ptr<const CameraFrame> Simulation::read_camera(CameraId id) const {
    const auto raw = static_cast<std::size_t>(id);
    if (raw >= kCameras.size()) {
        std::cerr << "mfr3duo_mujoco: invalid camera id\n";
        return nullptr;
    }
    mujoco_simulation::CameraStates states;
    if (!impl_->simulation.read_state(states) || !states || raw >= states->size() ||
        !(*states)[raw] || (*states)[raw]->id != raw)
        return nullptr;
    return std::shared_ptr<const CameraFrame>(
        new CameraFrame(std::make_shared<const CameraFrame::Impl>((*states)[raw])));
}
bool Simulation::read_contacts(ContactStates& contacts) const {
    mujoco_simulation::ContactStates lower;
    if (!impl_->simulation.read_contacts(lower)) return false;
    contacts.clear();
    contacts.reserve(lower.size());
    for (const auto& c : lower) contacts.push_back({c.geom1, c.geom2, c.distance});
    return true;
}
SimulationStatus Simulation::status() const {
    switch (impl_->simulation.status()) {
        case mujoco_simulation::SimulationStatus::Stopped:
            return SimulationStatus::Stopped;
        case mujoco_simulation::SimulationStatus::Running:
            return SimulationStatus::Running;
        case mujoco_simulation::SimulationStatus::Paused:
            return SimulationStatus::Paused;
        case mujoco_simulation::SimulationStatus::Stopping:
            return SimulationStatus::Stopping;
        case mujoco_simulation::SimulationStatus::Error:
            return SimulationStatus::Error;
        default:
            return SimulationStatus::Uninitialized;
    }
}
double Simulation::time() const { return impl_->simulation.time(); }
std::uint64_t Simulation::step_count() const { return impl_->simulation.step_count(); }
}  // namespace mfr3duo_mujoco
