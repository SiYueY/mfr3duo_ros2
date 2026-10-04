#include "grasp_recovery.hpp"
#include <chrono>
#include <cmath>
#include <thread>
namespace mfr3duo_robot {
namespace {
Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& p) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    t.linear() =
        Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
            .toRotationMatrix();
    return t;
}
geometry_msgs::msg::Pose message(const Eigen::Isometry3d& t) {
    geometry_msgs::msg::Pose p;
    p.position.x = t.translation().x();
    p.position.y = t.translation().y();
    p.position.z = t.translation().z();
    Eigen::Quaterniond q(t.linear());
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}
bool valid_pose(const geometry_msgs::msg::Pose& p) {
    const auto& q = p.orientation;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) &&
           std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) < 1e-5;
}
}  // namespace
GraspRecoveryResult recover_grasp(
    const rclcpp::Node::SharedPtr& node, mfr3duo_moveit::PlanningSceneInterface& scene,
    GraspObserver& observer, const std::string& id, Manipulator hand,
    const Eigen::Isometry3d& held_relative, const moveit_msgs::msg::CollisionObject& geometry,
    bool motion_confirmed, mfr3duo_control::Control& control, bool grasp_verified,
    double expected_width, double inner, double outer) {
    const auto unknown = [](std::string reason) {
        return GraspRecoveryResult{PhysicalHolding::Unknown, false, std::move(reason)};
    };
    if (!node || !motion_confirmed) return unknown("owned motion termination not confirmed");
    if (id.empty() || (hand != Manipulator::Left && hand != Manipulator::Right) ||
        geometry.primitives.size() != 1 || geometry.primitive_poses.size() != 1)
        return unknown("invalid recovery identity or unsupported geometry");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    bool all_held = true, all_released = true;
    GraspObservation observation;
    do {
        observation = observer.observe(id, hand);
        if (!observation.valid || !observation.object_visible ||
            observation.object_pose.header != observation.tool_pose.header ||
            observation.object_pose.header.frame_id.empty() ||
            observation.object_pose.header.stamp.sec < 0 ||
            !valid_pose(observation.object_pose.pose) || !valid_pose(observation.tool_pose.pose))
            return unknown("missing/incoherent physical observation");
        const auto age =
            (node->now() - rclcpp::Time(observation.object_pose.header.stamp)).seconds();
        if (age < -.1 || age > .3) return unknown("stale physical recovery observation");
        const auto relative = transform(observation.tool_pose.pose).inverse() *
                              transform(observation.object_pose.pose);
        const double distance = (relative.translation() - held_relative.translation()).norm();
        const double angle =
            Eigen::AngleAxisd(relative.linear() * held_relative.linear().transpose()).angle();
        const auto width = control.get_gripper_width(
            hand == Manipulator::Left ? mfr3duo_control::Gripper::Left
                                      : mfr3duo_control::Gripper::Right);
        if (!width) return unknown("missing/stale measured gripper opening");
        all_held = all_held && grasp_verified && width.value > expected_width - inner &&
                   width.value < expected_width + outer && distance <= .005 && angle <= .05;
        // A previously observed held transform must actually separate. An open command
        // or static proximity alone cannot establish release or continued holding.
        all_released = all_released && distance > .01;
        if (!all_held && !all_released) return unknown("physical holding remains ambiguous");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    bool attached = false;
    auto queried = scene.is_attached(id, attached);
    if (!queried) return unknown("attached query: " + queried.message);
    const auto arm = hand == Manipulator::Left ? mfr3duo_moveit::RobotGroup::LeftArm
                                               : mfr3duo_moveit::RobotGroup::RightArm;
    if (all_held) {
        if (!attached) return unknown("physical object held but attached representation missing");
        auto restored = scene.set_grasp_contact_allowed(id, arm, false);
        if (!restored) return unknown("held contact restoration: " + restored.message);
        return {PhysicalHolding::Held, true, "still held; retain attached representation"};
    }
    if (attached) {
        auto detached = scene.detach_object(id);
        if (!detached) return unknown("released detach: " + detached.message);
    }
    auto updated = geometry;
    updated.id = id;
    updated.operation = updated.ADD;
    updated.header.frame_id =
        hand == Manipulator::Left ? "left_fr3v2_1_hand_tcp" : "right_fr3v2_1_hand_tcp";
    updated.header.stamp = builtin_interfaces::msg::Time();
    updated.pose = geometry_msgs::msg::Pose();
    updated.primitive_poses = {message(
        transform(observation.tool_pose.pose).inverse() * transform(observation.object_pose.pose))};
    auto applied = scene.add_collision_object(updated);
    if (!applied) return unknown("released world restore: " + applied.message);
    auto restored = scene.set_grasp_contact_allowed(id, arm, false);
    if (!restored) return unknown("released contact restoration: " + restored.message);
    return {
        PhysicalHolding::Released, true, "released/lost; world pose restored from new observation"};
}
}  // namespace mfr3duo_robot
