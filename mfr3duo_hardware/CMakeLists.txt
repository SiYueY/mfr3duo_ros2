cmake_minimum_required(VERSION 3.16)
project(mfr3duo_hardware VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mfr3duo_mujoco CONFIG REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

if(BUILD_TESTING)
  add_executable(mfr3duo_mujoco_link_test test/link_mujoco.cpp)
  target_compile_features(mfr3duo_mujoco_link_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_mujoco_link_test
    PRIVATE mfr3duo_mujoco::mfr3duo_mujoco)
  add_test(NAME mfr3duo_mujoco_link_test COMMAND mfr3duo_mujoco_link_test)
endif()
ament_package()
