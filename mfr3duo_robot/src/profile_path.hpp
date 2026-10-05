#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace mfr3duo_robot {
inline std::string profile_path(
    const rclcpp::Node::SharedPtr& node, const char* parameter, const std::string& fallback) {
    if (!node->has_parameter(parameter)) node->declare_parameter<std::string>(parameter, fallback);
    return node->get_parameter(parameter).as_string();
}
}  // namespace mfr3duo_robot
