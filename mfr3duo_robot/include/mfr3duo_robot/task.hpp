#pragma once
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include "mfr3duo_robot/types.hpp"
namespace mfr3duo_robot {
// Mutable input, not thread-safe. execute snapshots it; start transfers ownership.
class RobotTask {
public:
    virtual ~RobotTask() = default;
    virtual std::unique_ptr<RobotTask> clone() const = 0;
    virtual std::string_view name() const noexcept = 0;
    void set_timeout(std::chrono::milliseconds timeout) { timeout_ = timeout; }
    std::optional<std::chrono::milliseconds> timeout() const noexcept { return timeout_; }

private:
    std::optional<std::chrono::milliseconds> timeout_;
};
}  // namespace mfr3duo_robot
