#include "component_ids.hpp"
#include "mfr3duo_mujoco/config.hpp"
#include "model_contract.hpp"
#include "robot_command.hpp"
#include "robot_state.hpp"
#include <gtest/gtest.h>
TEST(RobotCommand, GeneratesExactlyTheActiveJointDomain) {
    mfr3duo_mujoco::RobotCommand input;
    input.tmr.front_steering_position = 1.0;
    input.tmr.front_drive_velocity = 2.0;
    input.tmr.rear_steering_position = 3.0;
    input.tmr.rear_drive_velocity = 4.0;
    mujoco_simulation::RobotCommand output;
    ASSERT_TRUE(mfr3duo_mujoco::to_runtime_command(input, output));
    ASSERT_EQ(output.joints.size(), 21u);
    EXPECT_EQ(output.joints[15].id, mfr3duo_mujoco::kFrontSteering);
    EXPECT_EQ(output.joints[16].id, mfr3duo_mujoco::kFrontDrive);
    EXPECT_EQ(
        output.joints[15].mode, static_cast<std::uint8_t>(mujoco_simulation::JointMode::Position));
    EXPECT_EQ(
        output.joints[16].mode, static_cast<std::uint8_t>(mujoco_simulation::JointMode::Velocity));
    for (const auto& command : output.joints) {
        EXPECT_NE(command.id, mfr3duo_mujoco::kRocker);
        EXPECT_NE(command.id, mfr3duo_mujoco::kLeftFinger2);
        EXPECT_NE(command.id, mfr3duo_mujoco::kRightFinger2);
    }
}
TEST(ModelContract, FreezesCameraAndJointOrder) {
    EXPECT_EQ(mfr3duo_mujoco::kJoints.size(), 28u);
    EXPECT_EQ(mfr3duo_mujoco::kJoints[0].joint, "left_fr3v2_1_joint1");
    EXPECT_EQ(mfr3duo_mujoco::kJoints[27].joint, "right_fr3v2_1_finger_joint2");
    EXPECT_EQ(mfr3duo_mujoco::kCameras.size(), 14u);
    EXPECT_EQ(mfr3duo_mujoco::kCameras[8].camera, "d435_left_rgb");
}
TEST(SimulationConfig, ContainsOnlyTheModelPath) {
    mfr3duo_mujoco::SimulationConfig input;
    input.model_path = "/tmp/model.xml";
    EXPECT_EQ(input.model_path, "/tmp/model.xml");
}
TEST(RobotState, MapsFrozenOrderAndDoesNotInventBaseState) {
    mujoco_simulation::RobotState input;
    auto joints = std::make_shared<
        std::vector<mujoco_simulation::StateSnapshot<mujoco_simulation::JointState>>>();
    for (std::size_t id = 0; id < 28; ++id) {
        auto joint = std::make_shared<mujoco_simulation::JointState>();
        joint->id = id;
        joint->position = static_cast<double>(id);
        joint->velocity = static_cast<double>(id) + .5;
        joints->push_back(joint);
    }
    input.joints = joints;
    auto imus = std::make_shared<
        std::vector<mujoco_simulation::StateSnapshot<mujoco_simulation::ImuState>>>();
    auto imu = std::make_shared<mujoco_simulation::ImuState>();
    imu->id = 0;
    imu->orientation = {1., 2., 3., 4.};
    imus->push_back(imu);
    input.imus = imus;
    input.simulation_time = 2.0;
    input.step = 9;
    mfr3duo_mujoco::RobotState output;
    ASSERT_TRUE(mfr3duo_mujoco::from_runtime_state(input, output));
    EXPECT_EQ(output.left_arm.joints[0].position, 0.0);
    EXPECT_EQ(output.right_arm.joints[0].position, 7.0);
    EXPECT_EQ(output.spine.joint.position, 14.0);
    EXPECT_EQ(output.tmr.front_drive.position, 16.0);
    EXPECT_EQ(output.left_gripper.right_finger.position, 25.0);
    EXPECT_EQ(output.tmr.base.position, mfr3duo_mujoco::Vector3d{});
    EXPECT_EQ(output.imu.orientation[0], 4.0);
    EXPECT_EQ(output.imu.orientation[1], 1.0);
}
