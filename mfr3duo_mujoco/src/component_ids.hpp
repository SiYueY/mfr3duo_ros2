#pragma once
#include <array>
#include <cstddef>
namespace mfr3duo_mujoco {
inline constexpr std::size_t kLeftArmFirst = 0, kRightArmFirst = 7, kSpine = 14,
                             kFrontSteering = 15, kFrontDrive = 16, kRearSteering = 17,
                             kRearDrive = 18, kRocker = 19, kFrontCasterSteering = 20,
                             kFrontCasterRolling = 21, kRearCasterSteering = 22,
                             kRearCasterRolling = 23, kLeftFinger1 = 24, kLeftFinger2 = 25,
                             kRightFinger1 = 26, kRightFinger2 = 27;
inline constexpr std::array<std::size_t, 21> kActiveJointIds{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 24, 26};
}  // namespace mfr3duo_mujoco
