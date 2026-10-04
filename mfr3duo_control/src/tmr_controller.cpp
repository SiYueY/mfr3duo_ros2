#include "tmr_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <unordered_set>
#include <rclcpp/create_publisher.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace mfr3duo_control {
namespace {
std::int64_t monotonic_ns() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

controller_interface::CallbackReturn TmrController::on_init() {
    try {
        auto_declare<double>("wheel_radius", 0.05);
        auto_declare<double>("command_timeout", 0.5);
        auto_declare<double>("steering_slow_threshold", 0.1);
        auto_declare<double>("steering_stop_threshold", 0.6);
        auto_declare<double>("max_steering_rate", 1.5);
        auto_declare<double>("max_wheel_velocity", 8.0);
        auto_declare<double>("max_linear_velocity", 0.3);
        auto_declare<double>("max_angular_velocity", 0.5);
        auto_declare<double>("linear_acceleration", 0.5);
        auto_declare<double>("angular_acceleration", 0.5);
        auto_declare<std::string>("odom_frame", "odom");
        auto_declare<std::string>("base_frame", "base_link");
        auto_declare<bool>("publish_odom", true);
        auto_declare<bool>("publish_tf", true);
        auto_declare<double>("odom_publish_rate", 50.0);
        auto_declare<double>("diagnostic_publish_rate", 1.0);
        auto_declare<double>("odometry.maximum_condition", 1e6);
        auto_declare<double>("odometry.maximum_timestamp_gap", .5);
        auto_declare<double>("odometry.maximum_encoder_delta", .25);
        auto_declare<double>("odometry.steering_interval", .1);
        auto_declare<double>("odometry.residual_ratio", .2);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto prefix = "modules.module_" + std::to_string(i) + ".";
            auto_declare<std::string>(
                prefix + "steering_joint", "tmrv0_2_joint_" + std::to_string(2 * i));
            auto_declare<std::string>(
                prefix + "drive_joint", "tmrv0_2_joint_" + std::to_string(2 * i + 1));
            auto_declare<double>(prefix + "x", i == 0 ? 0.3 : -0.3);
            auto_declare<double>(prefix + "y", i == 0 ? -0.2 : 0.2);
            auto_declare<double>(prefix + "steering_offset", 0.0);
            auto_declare<double>(prefix + "steering_sign", 1.0);
            auto_declare<double>(prefix + "drive_sign", 1.0);
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TmrController::on_configure(const rclcpp_lifecycle::State&) {
    auto node = get_node();
    radius_ = node->get_parameter("wheel_radius").as_double();
    timeout_ = node->get_parameter("command_timeout").as_double();
    slow_ = node->get_parameter("steering_slow_threshold").as_double();
    stop_ = node->get_parameter("steering_stop_threshold").as_double();
    steering_rate_ = node->get_parameter("max_steering_rate").as_double();
    max_wheel_velocity_ = node->get_parameter("max_wheel_velocity").as_double();
    max_linear_velocity_ = node->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = node->get_parameter("max_angular_velocity").as_double();
    linear_acceleration_ = node->get_parameter("linear_acceleration").as_double();
    angular_acceleration_ = node->get_parameter("angular_acceleration").as_double();
    for (double value :
         {timeout_, steering_rate_, max_wheel_velocity_, max_linear_velocity_,
          max_angular_velocity_, linear_acceleration_, angular_acceleration_}) {
        if (!std::isfinite(value) || value <= 0.0)
            return controller_interface::CallbackReturn::ERROR;
    }
    if (!std::isfinite(slow_) || !std::isfinite(stop_) || slow_ < 0.0 || stop_ <= slow_ ||
        stop_ > kPi / 2.0)
        return controller_interface::CallbackReturn::ERROR;
    std::unordered_set<std::string> names;
    for (std::size_t i = 0; i < 2; ++i) {
        const auto prefix = "modules.module_" + std::to_string(i) + ".";
        steering_names_[i] = node->get_parameter(prefix + "steering_joint").as_string();
        drive_names_[i] = node->get_parameter(prefix + "drive_joint").as_string();
        if (steering_names_[i].empty() || drive_names_[i].empty() ||
            !names.insert(steering_names_[i]).second || !names.insert(drive_names_[i]).second) {
            return controller_interface::CallbackReturn::ERROR;
        }
        auto& m = modules_[i];
        m.x = node->get_parameter(prefix + "x").as_double();
        m.y = node->get_parameter(prefix + "y").as_double();
        m.steering_offset = node->get_parameter(prefix + "steering_offset").as_double();
        m.steering_sign = node->get_parameter(prefix + "steering_sign").as_double();
        m.drive_sign = node->get_parameter(prefix + "drive_sign").as_double();
    }
    if (!TmrKinematics::valid_geometry(modules_, radius_))
        return controller_interface::CallbackReturn::ERROR;
    OdometryLimits limits;
    limits.maximum_condition = node->get_parameter("odometry.maximum_condition").as_double();
    limits.maximum_timestamp_gap =
        node->get_parameter("odometry.maximum_timestamp_gap").as_double();
    limits.maximum_encoder_delta =
        node->get_parameter("odometry.maximum_encoder_delta").as_double();
    limits.steering_interval = node->get_parameter("odometry.steering_interval").as_double();
    limits.residual_ratio = node->get_parameter("odometry.residual_ratio").as_double();
    const auto odom_frame = node->get_parameter("odom_frame").as_string();
    const auto base_frame = node->get_parameter("base_frame").as_string();
    const double odom_rate = node->get_parameter("odom_publish_rate").as_double();
    const double diagnostic_rate = node->get_parameter("diagnostic_publish_rate").as_double();
    if (!odometry_.configure(modules_, radius_, limits) || odom_frame.empty() ||
        base_frame.empty() || odom_frame == base_frame || !std::isfinite(odom_rate) ||
        odom_rate < .01 || odom_rate > 1000 || !std::isfinite(diagnostic_rate) ||
        diagnostic_rate < .01 || diagnostic_rate > 100) {
        RCLCPP_ERROR(
            node->get_logger(), "Invalid odometry geometry, limits, frames or publishing rates");
        return controller_interface::CallbackReturn::ERROR;
    }
    odom_period_ns_ = static_cast<std::int64_t>(1e9 / odom_rate);
    diagnostic_period_ns_ = static_cast<std::int64_t>(1e9 / diagnostic_rate);
    if (node->get_parameter("publish_odom").as_bool()) {
        odom_publisher_ =
            std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
                rclcpp::create_publisher<nav_msgs::msg::Odometry>(node, "~/odom", rclcpp::QoS(10)));
        odom_publisher_->msg_.header.frame_id = odom_frame;
        odom_publisher_->msg_.child_frame_id = base_frame;
        odom_publisher_->msg_.pose.pose.orientation.w = 1;
    }
    if (node->get_parameter("publish_tf").as_bool()) {
        tf_publisher_ =
            std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
                rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(node, "tf", rclcpp::QoS(100)));
        tf_publisher_->msg_.transforms.resize(1);
        tf_publisher_->msg_.transforms[0].header.frame_id = odom_frame;
        tf_publisher_->msg_.transforms[0].child_frame_id = base_frame;
        tf_publisher_->msg_.transforms[0].transform.rotation.w = 1;
    }
    diagnostic_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
            rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                node, "diagnostics", rclcpp::QoS(10)));
    auto& diagnostic = diagnostic_publisher_->msg_;
    diagnostic.status.resize(1);
    diagnostic.status[0].name = node->get_name() + std::string("/odometry");
    diagnostic.status[0].hardware_id = "tmr";
    diagnostic.status[0].message.reserve(64);
    diagnostic.status[0].values.resize(3);
    diagnostic.status[0].values[0].key = "geometry_condition";
    diagnostic.status[0].values[0].value = std::to_string(odometry_.estimate().condition);
    diagnostic.status[0].values[1].key = "residual_ratio_limit";
    diagnostic.status[0].values[1].value = std::to_string(limits.residual_ratio);
    diagnostic.status[0].values[2].key = "encoder_delta_limit";
    diagnostic.status[0].values[2].value = std::to_string(limits.maximum_encoder_delta);
    subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::QoS(1),
        [this, max_linear = max_linear_velocity_,
         max_angular = max_angular_velocity_](const geometry_msgs::msg::Twist& msg) {
            const auto received_ns = monotonic_ns();
            if (!active_.load(std::memory_order_acquire)) return;
            if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.linear.y) ||
                !std::isfinite(msg.angular.z) || !std::isfinite(msg.linear.z) ||
                !std::isfinite(msg.angular.x) || !std::isfinite(msg.angular.y) ||
                msg.linear.z != 0.0 || msg.angular.x != 0.0 || msg.angular.y != 0.0)
                return;
            // Invalid planar inputs do not refresh the watchdog.
            if (std::hypot(msg.linear.x, msg.linear.y) > max_linear ||
                std::abs(msg.angular.z) > max_angular)
                return;
            command_.writeFromNonRT({msg.linear.x, msg.linear.y, msg.angular.z, received_ns});
        });
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TmrController::command_interface_configuration()
    const {
    controller_interface::InterfaceConfiguration config{
        controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (std::size_t i = 0; i < 2; ++i) {
        config.names.push_back(steering_names_[i] + "/position");
        config.names.push_back(drive_names_[i] + "/velocity");
    }
    return config;
}

controller_interface::InterfaceConfiguration TmrController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config{
        controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (std::size_t i = 0; i < 2; ++i) {
        config.names.push_back(steering_names_[i] + "/position");
        config.names.push_back(drive_names_[i] + "/position");
        config.names.push_back(drive_names_[i] + "/velocity");
    }
    return config;
}

controller_interface::CallbackReturn TmrController::on_activate(const rclcpp_lifecycle::State&) {
    if (command_interfaces_.size() != 4 || state_interfaces_.size() != 6)
        return controller_interface::CallbackReturn::ERROR;
    const auto locate = [](const auto& interfaces, const std::string& name, std::size_t& index) {
        for (std::size_t i = 0; i < interfaces.size(); ++i) {
            if (interfaces[i].get_name() == name) {
                index = i;
                return true;
            }
        }
        return false;
    };
    for (std::size_t i = 0; i < 2; ++i) {
        if (!locate(command_interfaces_, steering_names_[i] + "/position", steering_command_[i]) ||
            !locate(command_interfaces_, drive_names_[i] + "/velocity", drive_command_[i]) ||
            !locate(state_interfaces_, steering_names_[i] + "/position", steering_state_[i]) ||
            !locate(state_interfaces_, drive_names_[i] + "/position", drive_position_state_[i]) ||
            !locate(state_interfaces_, drive_names_[i] + "/velocity", drive_state_[i]))
            return controller_interface::CallbackReturn::ERROR;
        last_steering_[i] = state_interfaces_[steering_state_[i]].get_value();
        if (!std::isfinite(last_steering_[i]) ||
            !std::isfinite(state_interfaces_[drive_position_state_[i]].get_value()))
            return controller_interface::CallbackReturn::ERROR;
    }
    last_velocity_.fill(0.0);
    holding_ = true;
    hold();
    activated_ns_ = monotonic_ns();
    odometry_.reset_baseline();
    pending_diagnostic_ = OdometryStatus::Initialized;
    odom_confidence_ = 1;
    last_odom_ns_ = last_diagnostic_ns_ = 0;
    active_.store(true, std::memory_order_release);
    return controller_interface::CallbackReturn::SUCCESS;
}

void TmrController::hold() noexcept {
    // The backend clips position commands to [-pi, pi], but measures continuous
    // angles. Keep slew state continuous and encode only the hardware target.
    for (std::size_t i = 0; i < 2; ++i) {
        command_interfaces_[steering_command_[i]].set_value(
            TmrKinematics::shortest_angle(last_steering_[i]));
        command_interfaces_[drive_command_[i]].set_value(0.0);
    }
}

controller_interface::CallbackReturn TmrController::on_deactivate(const rclcpp_lifecycle::State&) {
    active_.store(false, std::memory_order_release);
    if (command_interfaces_.size() == 4 && state_interfaces_.size() == 6) {
        for (std::size_t i = 0; i < 2; ++i) {
            const double measured = state_interfaces_[steering_state_[i]].get_value();
            if (std::isfinite(measured)) last_steering_[i] = measured;
        }
        hold();
    }
    last_velocity_.fill(0.0);
    odometry_.reset_baseline();
    return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TmrController::on_cleanup(
    const rclcpp_lifecycle::State& state) {
    on_deactivate(state);
    subscription_.reset();
    odom_publisher_.reset();
    tf_publisher_.reset();
    diagnostic_publisher_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TmrController::on_error(const rclcpp_lifecycle::State& state) {
    return on_cleanup(state);
}

controller_interface::return_type TmrController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!active_.load(std::memory_order_acquire)) return controller_interface::return_type::OK;
    update_odometry(time);
    std::array<double, 2> steering{};
    for (std::size_t i = 0; i < 2; ++i) {
        steering[i] = state_interfaces_[steering_state_[i]].get_value();
        if (!std::isfinite(steering[i]) ||
            !std::isfinite(state_interfaces_[drive_state_[i]].get_value())) {
            hold();
            return controller_interface::return_type::ERROR;
        }
    }
    const auto input = *command_.readFromRT();
    const double age = static_cast<double>(monotonic_ns() - input.received_ns) * 1e-9;
    const double dt = period.seconds();
    const bool stale = input.received_ns <= activated_ns_ || age < 0.0 || age > timeout_;
    const bool zero = std::abs(input.vx) + std::abs(input.vy) + std::abs(input.wz) < 1e-9;
    if (stale || zero || !std::isfinite(dt) || dt <= 0.0) {
        if (!holding_) last_steering_ = steering;
        holding_ = true;
        last_velocity_.fill(0.0);
        hold();
        return controller_interface::return_type::OK;
    }
    holding_ = false;
    const std::array<double, 3> requested{input.vx, input.vy, input.wz};
    // Uniform linear acceleration limiting preserves the requested translation direction.
    const double dvx = requested[0] - last_velocity_[0];
    const double dvy = requested[1] - last_velocity_[1];
    const double change = std::hypot(dvx, dvy);
    const double scale = change > 0.0 ? std::min(1.0, linear_acceleration_ * dt / change) : 1.0;
    last_velocity_[0] += dvx * scale;
    last_velocity_[1] += dvy * scale;
    last_velocity_[2] += std::clamp(
        requested[2] - last_velocity_[2], -angular_acceleration_ * dt, angular_acceleration_ * dt);
    std::array<ModuleCommand, 2> output{};
    if (!TmrKinematics::inverse(
            modules_, radius_, last_velocity_[0], last_velocity_[1], last_velocity_[2], steering,
            output)) {
        hold();
        return controller_interface::return_type::ERROR;
    }
    double saturation = 1.0;
    for (const auto& m : output) {
        if (std::abs(m.drive) > max_wheel_velocity_)
            saturation = std::min(saturation, max_wheel_velocity_ / std::abs(m.drive));
    }
    // One scale for both wheels preserves the desired body twist ratio.
    double alignment = 1.0;
    for (std::size_t i = 0; i < 2; ++i)
        alignment = std::min(
            alignment, TmrKinematics::drive_scale(output[i].steering - steering[i], slow_, stop_));
    for (std::size_t i = 0; i < 2; ++i) {
        const double change = steering_rate_ * dt;
        last_steering_[i] += std::clamp(output[i].steering - last_steering_[i], -change, change);
        command_interfaces_[steering_command_[i]].set_value(
            TmrKinematics::shortest_angle(last_steering_[i]));
        command_interfaces_[drive_command_[i]].set_value(output[i].drive * saturation * alignment);
    }
    return controller_interface::return_type::OK;
}

void TmrController::update_odometry(const rclcpp::Time& time) noexcept {
    const std::array<double, 2> steering{
        state_interfaces_[steering_state_[0]].get_value(),
        state_interfaces_[steering_state_[1]].get_value()};
    const std::array<double, 2> wheels{
        state_interfaces_[drive_position_state_[0]].get_value(),
        state_interfaces_[drive_position_state_[1]].get_value()};
    const auto& estimate = odometry_.update(steering, wheels, time.nanoseconds());
    constexpr std::array<unsigned, 7> severity{1, 0, 2, 2, 3, 1, 2};
    if (severity[static_cast<unsigned>(estimate.status)] >=
        severity[static_cast<unsigned>(pending_diagnostic_)])
        pending_diagnostic_ = estimate.status;
    odom_confidence_ = std::min(odom_confidence_, estimate.confidence);
    const auto stamp = time.nanoseconds();
    const bool odom_due =
        last_odom_ns_ == 0 || stamp < last_odom_ns_ || stamp - last_odom_ns_ >= odom_period_ns_;
    if (odom_due && stamp >= 0) {
        const double qz = std::sin(estimate.yaw / 2), qw = std::cos(estimate.yaw / 2);
        if (odom_publisher_ && odom_publisher_->trylock()) {
            auto& message = odom_publisher_->msg_;
            message.header.stamp = time;
            message.pose.pose.position.x = estimate.x;
            message.pose.pose.position.y = estimate.y;
            message.pose.pose.orientation.z = qz;
            message.pose.pose.orientation.w = qw;
            message.twist.twist.linear.x = estimate.vx;
            message.twist.twist.linear.y = estimate.vy;
            message.twist.twist.angular.z = estimate.wz;
            message.pose.covariance.fill(0);
            message.twist.covariance.fill(0);
            const double variance =
                odom_confidence_ > 0 ? .0001 / (odom_confidence_ * odom_confidence_) : 1e6;
            for (std::size_t index : {0U, 7U, 35U}) {
                message.pose.covariance[index] = variance;
                message.twist.covariance[index] = variance * 4;
            }
            for (std::size_t index : {14U, 21U, 28U}) {
                message.pose.covariance[index] = 1e6;
                message.twist.covariance[index] = 1e6;
            }
            odom_publisher_->unlockAndPublish();
            odom_confidence_ = 1;
        }
        if (tf_publisher_ && tf_publisher_->trylock()) {
            auto& transform = tf_publisher_->msg_.transforms[0];
            transform.header.stamp = time;
            transform.transform.translation.x = estimate.x;
            transform.transform.translation.y = estimate.y;
            transform.transform.rotation.z = qz;
            transform.transform.rotation.w = qw;
            tf_publisher_->unlockAndPublish();
        }
        last_odom_ns_ = stamp;
    }
    if (diagnostic_publisher_ &&
        (last_diagnostic_ns_ == 0 || stamp < last_diagnostic_ns_ ||
         stamp - last_diagnostic_ns_ >= diagnostic_period_ns_) &&
        diagnostic_publisher_->trylock()) {
        constexpr std::array<const char*, 7> descriptions{
            "Encoder baseline",
            "Valid encoder odometry",
            "ROS clock reset",
            "Encoder reset",
            "Invalid measured state",
            "Large steering interval",
            "Module disagreement or slip"};
        auto& message = diagnostic_publisher_->msg_;
        message.header.stamp = time;
        auto& status = message.status[0];
        status.message = descriptions[static_cast<unsigned>(pending_diagnostic_)];
        status.level = pending_diagnostic_ == OdometryStatus::Valid          ? status.OK
                       : pending_diagnostic_ == OdometryStatus::InvalidState ? status.ERROR
                                                                             : status.WARN;
        diagnostic_publisher_->unlockAndPublish();
        last_diagnostic_ns_ = stamp;
        pending_diagnostic_ = OdometryStatus::Valid;
    }
}

}  // namespace mfr3duo_control

PLUGINLIB_EXPORT_CLASS(mfr3duo_control::TmrController, controller_interface::ControllerInterface)
