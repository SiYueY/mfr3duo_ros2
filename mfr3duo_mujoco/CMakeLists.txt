cmake_minimum_required(VERSION 3.16)
project(mfr3duo_mujoco VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco_simulation CONFIG REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/runtime_config_file.cpp
  src/robot_command.cpp
  src/robot_state.cpp
  src/simulation.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 0)
target_compile_definitions(${PROJECT_NAME} PRIVATE MFR3DUO_MUJOCO_BUILDING_LIBRARY)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_index_cpp::ament_index_cpp
  mujoco_simulation::mujoco_simulation)

add_executable(mfr3duo_mujoco_viewer
  src/mfr3duo_viewer.cpp
  src/runtime_config_file.cpp)
target_compile_features(mfr3duo_mujoco_viewer PRIVATE cxx_std_17)
target_include_directories(mfr3duo_mujoco_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mfr3duo_mujoco_viewer PRIVATE
  ament_index_cpp::ament_index_cpp
  mujoco_simulation::mujoco_simulation)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS mfr3duo_mujoco_viewer RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp mujoco_simulation)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_adapter test/test_adapter.cpp)
    target_link_libraries(test_adapter ${PROJECT_NAME})
    target_include_directories(test_adapter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ament_add_gtest(test_simulation test/test_simulation.cpp)
    target_link_libraries(test_simulation ${PROJECT_NAME})
    target_include_directories(test_simulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ament_add_gtest(test_viewer_config test/test_viewer_cli.cpp src/runtime_config_file.cpp)
    target_include_directories(test_viewer_config PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_viewer_config
      ament_index_cpp::ament_index_cpp
      mujoco_simulation::mujoco_simulation)
  endif()
endif()
ament_package()
