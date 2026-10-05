cmake_minimum_required(VERSION 3.16)
project(mfr3duo_nav VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
add_library(mfr3duo_nav SHARED src/navigator.cpp)
target_compile_features(mfr3duo_nav PUBLIC cxx_std_17)
target_compile_options(mfr3duo_nav PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(mfr3duo_nav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mfr3duo_nav rclcpp rclcpp_action geometry_msgs nav_msgs
  nav2_msgs lifecycle_msgs tf2_ros tf2_msgs)
install(TARGETS mfr3duo_nav EXPORT export_mfr3duo_nav
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_mfr3duo_nav HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs
  lifecycle_msgs tf2_ros tf2_msgs)
install(DIRECTORY config maps launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/simulation_localization.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME simulation_localization_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/simulation_localization_test.py)
  add_test(NAME navigator_config_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/config_contract_test.py)
  add_test(NAME navigator_installed_consumer COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/installed_consumer_test.py)
  set_tests_properties(navigator_installed_consumer PROPERTIES TIMEOUT 120)
  add_test(NAME navigator_runtime_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/runtime_test.py)
  set_tests_properties(navigator_runtime_test PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE)
  add_executable(navigator_action_probe test/action_probe.cpp)
  target_link_libraries(navigator_action_probe mfr3duo_nav)
  add_test(NAME navigator_action_probe COMMAND navigator_action_probe)
  set_tests_properties(navigator_action_probe PROPERTIES TIMEOUT 40)
  add_executable(navigator_builder_test test/builder_test.cpp)
  target_link_libraries(navigator_builder_test mfr3duo_nav)
  add_test(NAME navigator_builder_test COMMAND navigator_builder_test)
  set_tests_properties(navigator_builder_test PROPERTIES TIMEOUT 30)
  find_package(mfr3duo_moveit REQUIRED)
  add_executable(navigator_runtime_probe test/navigator_runtime_probe.cpp)
  target_link_libraries(navigator_runtime_probe mfr3duo_nav
    mfr3duo_moveit::mfr3duo_moveit)
  install(TARGETS navigator_runtime_probe DESTINATION lib/${PROJECT_NAME})
endif()
ament_package()
