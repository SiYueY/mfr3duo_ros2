#pragma once
#include "component_ids.hpp"
#include <array>
#include <string_view>
namespace mfr3duo_mujoco {

struct JointContract {
    std::size_t id;
    std::string_view joint;
    std::string_view actuator;
    bool active;
    bool prismatic;
};

inline constexpr std::array<JointContract, 28> kJoints{
    {{0, "left_fr3v2_1_joint1", "left_fr3v2_1_joint1_motor", true, false},
     {1, "left_fr3v2_1_joint2", "left_fr3v2_1_joint2_motor", true, false},
     {2, "left_fr3v2_1_joint3", "left_fr3v2_1_joint3_motor", true, false},
     {3, "left_fr3v2_1_joint4", "left_fr3v2_1_joint4_motor", true, false},
     {4, "left_fr3v2_1_joint5", "left_fr3v2_1_joint5_motor", true, false},
     {5, "left_fr3v2_1_joint6", "left_fr3v2_1_joint6_motor", true, false},
     {6, "left_fr3v2_1_joint7", "left_fr3v2_1_joint7_motor", true, false},
     {7, "right_fr3v2_1_joint1", "right_fr3v2_1_joint1_motor", true, false},
     {8, "right_fr3v2_1_joint2", "right_fr3v2_1_joint2_motor", true, false},
     {9, "right_fr3v2_1_joint3", "right_fr3v2_1_joint3_motor", true, false},
     {10, "right_fr3v2_1_joint4", "right_fr3v2_1_joint4_motor", true, false},
     {11, "right_fr3v2_1_joint5", "right_fr3v2_1_joint5_motor", true, false},
     {12, "right_fr3v2_1_joint6", "right_fr3v2_1_joint6_motor", true, false},
     {13, "right_fr3v2_1_joint7", "right_fr3v2_1_joint7_motor", true, false},
     {14, "franka_spine_vertical_joint", "franka_spine_motor", true, true},
     {15, "tmrv0_2_joint_0", "tmrv0_2_joint_0_motor", true, false},
     {16, "tmrv0_2_joint_1", "tmrv0_2_joint_1_motor", true, false},
     {17, "tmrv0_2_joint_2", "tmrv0_2_joint_2_motor", true, false},
     {18, "tmrv0_2_joint_3", "tmrv0_2_joint_3_motor", true, false},
     {19, "rocker_arm_joint", "", false, false},
     {20, "caster_front_left_steering_joint", "", false, false},
     {21, "caster_front_left_joint", "", false, false},
     {22, "caster_rear_right_steering_joint", "", false, false},
     {23, "caster_rear_right_joint", "", false, false},
     {24, "left_fr3v2_1_finger_joint1", "left_fr3v2_1_finger_motor", true, true},
     {25, "left_fr3v2_1_finger_joint2", "", false, true},
     {26, "right_fr3v2_1_finger_joint1", "right_fr3v2_1_finger_motor", true, true},
     {27, "right_fr3v2_1_finger_joint2", "", false, true}}};
struct CameraContract {
    std::string_view role, camera, frame, optical;
    bool rgb, depth;
};
inline constexpr std::array<CameraContract, 14> kCameras{
    {{"BaseFrontColor", "camera_front_color", "camera_front_color_frame",
      "camera_front_color_optical_frame", true, false},
     {"BaseFrontDepth", "camera_front_depth", "camera_front_depth_frame",
      "camera_front_depth_optical_frame", false, true},
     {"BaseRearColor", "camera_rear_color", "camera_rear_color_frame",
      "camera_rear_color_optical_frame", true, false},
     {"BaseRearDepth", "camera_rear_depth", "camera_rear_depth_frame",
      "camera_rear_depth_optical_frame", false, true},
     {"BaseLeftColor", "camera_left_color", "camera_left_color_frame",
      "camera_left_color_optical_frame", true, false},
     {"BaseLeftDepth", "camera_left_depth", "camera_left_depth_frame",
      "camera_left_depth_optical_frame", false, true},
     {"BaseRightColor", "camera_right_color", "camera_right_color_frame",
      "camera_right_color_optical_frame", true, false},
     {"BaseRightDepth", "camera_right_depth", "camera_right_depth_frame",
      "camera_right_depth_optical_frame", false, true},
     {"LeftWristColor", "d435_left_rgb", "left_d435_link", "left_d435_color_optical_frame", true,
      false},
     {"LeftWristDepth", "d435_left_depth", "left_d435_link", "left_d435_depth_optical_frame", false,
      true},
     {"RightWristColor", "d435_right_rgb", "right_d435_link", "right_d435_color_optical_frame",
      true, false},
     {"RightWristDepth", "d435_right_depth", "right_d435_link", "right_d435_depth_optical_frame",
      false, true},
     {"HeadLeft", "head_zed_left", "head_zed_left_camera_frame",
      "head_zed_left_camera_optical_frame", true, false},
     {"HeadRight", "head_zed_right", "head_zed_right_camera_frame",
      "head_zed_right_camera_optical_frame", true, false}}};
}  // namespace mfr3duo_mujoco
