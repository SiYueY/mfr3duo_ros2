#pragma once
#include "mfr3duo_robot/task.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <utility>
namespace mfr3duo_robot {
class PlaceTask final : public RobotTask {
public:
    explicit PlaceTask(std::string object_id) : object_id_(std::move(object_id)) {}
    const std::string& object_id() const noexcept { return object_id_; }
    void set_manipulator(Manipulator hand) noexcept { hand_ = hand; }
    Manipulator manipulator() const noexcept { return hand_; }
    // Desired resting object pose, rather than an unverified release command.
    void set_place_pose(geometry_msgs::msg::PoseStamped pose) { place_ = std::move(pose); }
    const std::optional<geometry_msgs::msg::PoseStamped>& place_pose() const noexcept {
        return place_;
    }
    std::string_view name() const noexcept override { return "PlaceTask"; }
    std::unique_ptr<RobotTask> clone() const override;

private:
    std::string object_id_;
    Manipulator hand_{Manipulator::Auto};
    std::optional<geometry_msgs::msg::PoseStamped> place_;
};
}  // namespace mfr3duo_robot
