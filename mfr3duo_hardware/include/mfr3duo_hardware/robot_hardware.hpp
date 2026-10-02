#pragma once

#include <chrono>
#include <memory>

#include "mfr3duo_hardware/robot_types.hpp"
#include "mfr3duo_hardware/visibility_control.hpp"

// RobotHardware is the single official whole-robot C++ hardware API of
// MFR3Duo. ROS 2 and MuJoCo are adapters and backends behind this interface;
// neither of them may define it.

namespace mfr3duo_hardware {

/**
 * @brief Runtime options of RobotHardware.
 *
 * control_period is the requested control cycle. The backend derives a fixed
 * number of physics steps from it during initialize() and never changes that
 * number while running.
 */
struct RobotHardwareOptions {
    std::chrono::nanoseconds control_period{std::chrono::milliseconds(2)};
    bool viewer_enabled{false};
};

/**
 * @brief Whole-robot hardware interface of MFR3Duo.
 *
 * Lifecycle:
 *
 * @code
 * Uninitialized --initialize--> Inactive --activate--> Active
 *      ^                            ^                      |
 *      |                            +------deactivate------+
 *      +---------------------shutdown-----------------------+
 * @endcode
 */
class MFR3DUO_HARDWARE_PUBLIC RobotHardware {
public:
    RobotHardware();
    ~RobotHardware();

    RobotHardware(RobotHardware&&) noexcept;
    RobotHardware& operator=(RobotHardware&&) noexcept;

    RobotHardware(const RobotHardware&) = delete;
    RobotHardware& operator=(const RobotHardware&) = delete;

    /** @brief Create the backend, load the robot and enter Inactive. */
    bool initialize(const RobotHardwareOptions& options);

    /** @brief Hold the current pose and start accepting commands. */
    bool activate();

    /** @brief Submit a safe hold command and stop accepting commands. */
    bool deactivate();

    /** @brief Release the backend. Idempotent. */
    bool shutdown();

    /**
     * @brief Run one control cycle of the backend and refresh RobotState.
     *
     * update() is not step(): it carries robot hardware semantics. A real robot
     * backend implements it by reading drivers instead of stepping a simulator.
     */
    bool update();

    /**
     * @brief Submit a complete whole-robot command.
     *
     * The command is validated, prepared and committed as a whole. When the
     * call returns false nothing has been committed.
     */
    bool write_command(const RobotCommand& command);

    /** @brief Read the coherent motion snapshot produced by update(). */
    bool read_state(RobotState& state) const;

    /** @brief Read the base IMU. */
    bool read_state(ImuState& state) const;

    /** @brief Read one 2D LiDAR. */
    bool read_state(Lidar id, LaserScan& scan) const;

    /** @brief Read one camera stream. */
    bool read_state(Camera id, CameraFrame& frame) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mfr3duo_hardware
