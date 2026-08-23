#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mfr3duo_mujoco/visibility_control.hpp"

namespace mfr3duo_mujoco {

enum class CameraId : std::uint8_t {
    BaseFrontColor,
    BaseFrontDepth,
    BaseRearColor,
    BaseRearDepth,
    BaseLeftColor,
    BaseLeftDepth,
    BaseRightColor,
    BaseRightDepth,
    LeftWristColor,
    LeftWristDepth,
    RightWristColor,
    RightWristDepth,
    HeadLeft,
    HeadRight,
};

/// Non-owning image bytes. The CameraFrame snapshot keeps the bytes alive.
struct ImageView {
    std::uint64_t timestamp{0};
    const std::uint8_t* data{nullptr};
    std::size_t size{0};
    std::uint32_t height{0};
    std::uint32_t width{0};
    std::uint32_t step{0};
    std::string encoding;
};

/// Small camera calibration and frame metadata copied into a CameraFrame.
struct CameraMetadata {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    std::string frame_id;
    std::string optical_frame_id;
    std::uint32_t height{0};
    std::uint32_t width{0};
    std::string distortion_model;
    std::vector<double> distortion;
};

/// Immutable high-bandwidth camera snapshot.
class MFR3DUO_MUJOCO_PUBLIC CameraFrame {
public:
    ~CameraFrame();
    CameraFrame(const CameraFrame&) = delete;
    CameraFrame& operator=(const CameraFrame&) = delete;

    const CameraMetadata& metadata() const;
    ImageView color() const;
    ImageView depth() const;

private:
    class Impl;
    explicit CameraFrame(std::shared_ptr<const Impl> impl);
    std::shared_ptr<const Impl> impl_;
    friend class Simulation;
};

}  // namespace mfr3duo_mujoco
