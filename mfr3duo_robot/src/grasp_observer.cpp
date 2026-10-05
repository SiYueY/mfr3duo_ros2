#include "grasp_observer.hpp"
#include "profile_path.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace mfr3duo_robot {
namespace {
using Pose = geometry_msgs::msg::PoseStamped;
using Samples = std::map<std::int64_t, Pose>;
std::int64_t stamp(const Pose& pose) {
    return static_cast<std::int64_t>(pose.header.stamp.sec) * 1000000000LL +
           pose.header.stamp.nanosec;
}
bool finite_pose(const Pose& p) {
    const auto& q = p.pose.orientation;
    return !p.header.frame_id.empty() && p.header.stamp.sec >= 0 &&
           p.header.stamp.nanosec < 1000000000U && std::isfinite(p.pose.position.x) &&
           std::isfinite(p.pose.position.y) && std::isfinite(p.pose.position.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
           std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) < 1e-5;
}
struct Cache {
    std::mutex mutex;
    std::condition_variable changed;
    std::map<std::string, Pose> objects;
    std::array<Samples, 2> tools;
};
std::optional<Pose> at_time(const Samples& samples, std::int64_t time) {
    auto after = samples.lower_bound(time);
    if (after != samples.end() && after->first == time) return after->second;
    if (after == samples.end() || after == samples.begin()) return std::nullopt;
    const auto before = std::prev(after);
    if (time - before->first > 50000000 || after->first - time > 50000000 ||
        before->second.header.frame_id != after->second.header.frame_id ||
        !finite_pose(before->second) || !finite_pose(after->second))
        return std::nullopt;
    const double t = static_cast<double>(time - before->first) / (after->first - before->first);
    auto result = before->second;
    result.header.stamp = rclcpp::Time(time);
    auto& p = result.pose.position;
    const auto& a = before->second.pose.position;
    const auto& b = after->second.pose.position;
    p.x = a.x + t * (b.x - a.x);
    p.y = a.y + t * (b.y - a.y);
    p.z = a.z + t * (b.z - a.z);
    const auto& qa = before->second.pose.orientation;
    const auto& qb = after->second.pose.orientation;
    const auto q = Eigen::Quaterniond(qa.w, qa.x, qa.y, qa.z)
                       .slerp(t, Eigen::Quaterniond(qb.w, qb.x, qb.y, qb.z));
    result.pose.orientation.x = q.x();
    result.pose.orientation.y = q.y();
    result.pose.orientation.z = q.z();
    result.pose.orientation.w = q.w();
    return result;
}
}  // namespace
struct SimulationGraspObserver::Impl {
    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callbacks;
    std::shared_ptr<Cache> cache = std::make_shared<Cache>();
    std::vector<rclcpp::Subscription<Pose>::SharedPtr> subscriptions;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    double timeout{.5}, age{.3};
    explicit Impl(rclcpp::Node::SharedPtr supplied)
    : node(std::move(supplied)), buffer(node->get_clock()), listener(buffer, node, false) {
        const auto profile = YAML::LoadFile(profile_path(
            node, "grasp_profile_path",
            ament_index_cpp::get_package_share_directory("mfr3duo_robot") + "/config/grasp.yaml"));
        const auto seconds = [&](const char* name, double fallback) {
            if (!node->has_parameter(name)) node->declare_parameter(name, fallback);
            const auto value = node->get_parameter(name).as_double();
            if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name);
            return value;
        };
        timeout =
            seconds("grasp_observation.timeout", profile["observation"]["timeout"].as<double>());
        age = seconds(
            "grasp_observation.maximum_age", profile["observation"]["maximum_age"].as<double>());
        const auto prefix = [&](const char* name, const std::string& fallback) {
            if (!node->has_parameter(name)) node->declare_parameter(name, fallback);
            return node->get_parameter(name).as_string();
        };
        const auto objects = prefix(
            "perception.objects_topic_prefix",
            profile["observation"]["objects_topic_prefix"].as<std::string>());
        const auto tools = prefix(
            "perception.tools_topic_prefix",
            profile["observation"]["tools_topic_prefix"].as<std::string>());
        callbacks = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
            [group = callbacks]() mutable { group.reset(); });
        const std::weak_ptr<Cache> weak = cache;
        rclcpp::SubscriptionOptions options;
        options.callback_group = callbacks;
        for (const auto& object : profile["observation"]["objects"]) {
            const auto id = object.first.as<std::string>();
            {
                std::lock_guard<std::mutex> lock(cache->mutex);
                cache->objects.emplace(id, Pose{});
            }
            subscriptions.push_back(node->create_subscription<Pose>(
                objects + "/" + id + "/pose", 10,
                [weak, id, clock = node->get_clock()](Pose::ConstSharedPtr message) {
                    if (auto c = weak.lock()) {
                        std::lock_guard<std::mutex> lock(c->mutex);
                        auto& latest = c->objects.at(id);
                        if (!finite_pose(latest) ||
                            stamp(latest) > clock->now().nanoseconds() + 100000000 ||
                            stamp(*message) >= stamp(latest))
                            latest = *message;
                        c->changed.notify_all();
                    }
                },
                options));
        }
        for (std::size_t hand = 0; hand < 2; ++hand) {
            subscriptions.push_back(node->create_subscription<Pose>(
                tools + (hand == 0 ? "/left/pose" : "/right/pose"), 10,
                [weak, hand](Pose::ConstSharedPtr message) {
                    if (auto c = weak.lock()) {
                        std::lock_guard<std::mutex> lock(c->mutex);
                        auto& samples = c->tools[hand];
                        samples[stamp(*message)] = *message;
                        while (samples.size() > 64) samples.erase(samples.begin());
                        c->changed.notify_all();
                    }
                },
                options));
        }
    }
};
SimulationGraspObserver::SimulationGraspObserver(const rclcpp::Node::SharedPtr& node) {
    if (!node) throw std::invalid_argument("observer requires external node");
    impl_ = std::make_unique<Impl>(node);
}
SimulationGraspObserver::~SimulationGraspObserver() = default;
GraspObservation SimulationGraspObserver::observe(std::string_view object_id, Manipulator hand) {
    GraspObservation result;
    const auto fail = [&](const char* reason) {
        result.valid = false;
        result.diagnostic = reason;
        return result;
    };
    if (object_id.empty() || (hand != Manipulator::Left && hand != Manipulator::Right))
        return fail("explicit manipulator and object ID required");
    const auto c = impl_->cache;
    std::unique_lock<std::mutex> lock(c->mutex);
    const auto found = c->objects.find(std::string(object_id));
    if (found == c->objects.end()) return fail("unknown object ID");
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(impl_->timeout);
    while (true) {
        result.object_pose = found->second;
        auto tool = at_time(c->tools[hand == Manipulator::Left ? 0 : 1], stamp(result.object_pose));
        if (tool) {
            result.tool_pose = *tool;
            break;
        }
        if (c->changed.wait_until(lock, deadline) == std::cv_status::timeout)
            return fail("object pose and tool pose cannot be matched in time");
    }
    lock.unlock();
    if (!finite_pose(result.object_pose) || !finite_pose(result.tool_pose))
        return fail("invalid perception pose");
    const auto age = (impl_->node->now() - rclcpp::Time(result.object_pose.header.stamp)).seconds();
    if (age < -.1 || age > impl_->age) return fail("stale perception pose");
    if (result.tool_pose.header.frame_id != result.object_pose.header.frame_id) {
        try {
            const auto tf = impl_->buffer.lookupTransform(
                result.object_pose.header.frame_id, result.tool_pose.header.frame_id,
                tf2::TimePoint(std::chrono::nanoseconds(stamp(result.object_pose))));
            const auto& r = tf.transform.rotation;
            const auto& t = tf.transform.translation;
            Eigen::Quaterniond rotation(r.w, r.x, r.y, r.z);
            const auto& p = result.tool_pose.pose.position;
            const auto& q = result.tool_pose.pose.orientation;
            const Eigen::Vector3d position =
                rotation * Eigen::Vector3d(p.x, p.y, p.z) + Eigen::Vector3d(t.x, t.y, t.z);
            const Eigen::Quaterniond orientation =
                rotation * Eigen::Quaterniond(q.w, q.x, q.y, q.z);
            result.tool_pose.pose.position.x = position.x();
            result.tool_pose.pose.position.y = position.y();
            result.tool_pose.pose.position.z = position.z();
            result.tool_pose.pose.orientation.x = orientation.x();
            result.tool_pose.pose.orientation.y = orientation.y();
            result.tool_pose.pose.orientation.z = orientation.z();
            result.tool_pose.pose.orientation.w = orientation.w();
        } catch (const tf2::TransformException&) {
            return fail("TF unavailable at object sampling time");
        }
    }
    result.tool_pose.header = result.object_pose.header;
    result.valid = finite_pose(result.tool_pose);
    result.object_visible = result.valid;
    result.diagnostic = "time-matched standard perception poses";
    return result;
}
}  // namespace mfr3duo_robot
