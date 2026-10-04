#include <chrono>
#include <iostream>
#include <type_traits>

#include "mfr3duo_nav/navigator.hpp"

static_assert(!std::is_copy_constructible_v<mfr3duo_nav::Navigator>);
static_assert(!std::is_move_constructible_v<mfr3duo_nav::Navigator>);
static_assert(std::is_copy_constructible_v<mfr3duo_nav::NavigationHandle>);
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    using mfr3duo_nav::ErrorCode;
    int failed = 0;
    auto node = rclcpp::Node::make_shared("navigator_builder_test");
    {
        mfr3duo_nav::NavigationHandle invalid;
        if (invalid.valid() || invalid.result() ||
            invalid.cancel().code != ErrorCode::InvalidGoal ||
            invalid.wait().code != ErrorCode::InvalidGoal)
            ++failed;
        mfr3duo_nav::Navigator nav(node);
        if (nav.is_ready() || !nav.cancel() ||
            nav.initialize(std::chrono::milliseconds(0)).code != ErrorCode::Timeout)
            ++failed;
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "map";
        target.pose.orientation.w = 1.;
        auto handle = nav.start_navigate_to(target);
        if (!handle.valid() || handle.wait().code != ErrorCode::NotInitialized ||
            !handle.result() || handle.result()->code != ErrorCode::NotInitialized)
            ++failed;
        auto retained = handle;
        if (retained.wait().code != ErrorCode::NotInitialized) ++failed;
        geometry_msgs::msg::PoseWithCovarianceStamped initial;
        if (nav.set_initial_pose(initial).code != ErrorCode::InvalidGoal) ++failed;
        initial.header.frame_id = "map";
        initial.pose.pose.orientation.w = 1.;
        if (!nav.set_initial_pose(initial)) ++failed;
        initial.pose.covariance[0] = -1.;
        if (nav.set_initial_pose(initial).code != ErrorCode::InvalidGoal) ++failed;
        if (nav.get_current_pose(target).code != ErrorCode::NotInitialized) ++failed;
    }
    for (int i = 0; i < 30; ++i) {
        mfr3duo_nav::Navigator temporary(node);
    }
    node.reset();
    rclcpp::shutdown();
    std::cout << "Navigator builder checks " << (failed ? "FAIL" : "PASS") << '\n';
    return failed ? 1 : 0;
}
