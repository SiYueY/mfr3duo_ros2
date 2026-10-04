#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mfr3duo_nav/types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mfr3duo_nav {

class NavigationHandle {
public:
    NavigationHandle() = default;
    bool valid() const noexcept;
    NavigationState state() const;
    Result cancel();
    Result wait();
    std::optional<Result> result() const;

private:
    friend class Navigator;
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

/** The application spins the supplied node; Navigator owns no executor/thread. */
class Navigator {
public:
    explicit Navigator(const rclcpp::Node::SharedPtr& node);
    ~Navigator();
    Navigator(const Navigator&) = delete;
    Navigator& operator=(const Navigator&) = delete;
    Navigator(Navigator&&) = delete;
    Navigator& operator=(Navigator&&) = delete;
    Result initialize(std::chrono::milliseconds timeout);
    bool is_ready() const;
    Result set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
    Result get_current_pose(geometry_msgs::msg::PoseStamped& pose) const;
    Result navigate_to(const geometry_msgs::msg::PoseStamped& target);
    Result navigate_through(const std::vector<geometry_msgs::msg::PoseStamped>& targets);
    NavigationHandle start_navigate_to(const geometry_msgs::msg::PoseStamped& target);
    Result cancel();
    NavigationState state() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mfr3duo_nav
