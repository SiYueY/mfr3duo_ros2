# 仓库架构

`mfr3duo_ros2` 是 colcon 集成仓库，目标平台为 Ubuntu 22.04 和 ROS 2 Humble。

```text
mfr3duo_description (ament_cmake, submodule)
    ├── mfr3duo_mujoco (纯 CMake, submodule)
    │       └── mfr3duo_hardware (ament_cmake)
    ├── mfr3duo_moveit (ament_cmake)
    └── mfr3duo_nav (ament_cmake)

mfr3duo_robot (ament_cmake)
    └── description + hardware + moveit + nav
```

`mfr3duo_description` 提供机器人模型和资源。`mfr3duo_mujoco` 保持独立纯 CMake 包，依赖外部安装的 `romujoco`，通过 `colcon.pkg` 声明对 description 的构建依赖，并仅在 colcon 构建中关闭 teleop。colcon 通过安装前缀发现依赖，不使用相邻 submodule 目录的 CMake 路径或 `add_subdirectory()`。

`mfr3duo_hardware` 分为两层。`libmfr3duo_hardware.so` 提供 `RobotHardware`，这是 MFR3Duo 唯一正式的整机 C++ API，封装 `initialize/activate/deactivate/shutdown/update/write_command/read_state`，只私有依赖 `mfr3duo_mujoco`，Public Header 不包含 ROS 或 MuJoCo 类型。`libmfr3duo_ros2_adapter.so` 提供 `Ros2ControlAdapter`（`hardware_interface::SystemInterface`，plugin 名 `mfr3duo_hardware/Ros2ControlAdapter`）与 `Ros2SensorAdapter`，二者共享同一个 `RobotHardware` 实例，依赖方向固定为 `mfr3duo_ros2_adapter → mfr3duo_hardware → mfr3duo_mujoco`。

ros2_control 契约保持不变：53 个 command interface、70 个 state interface，覆盖双臂、脊柱、TMR 主动关节、夹爪与 IMU。关节状态、IMU、双臂轨迹、TMR 转向/驱动和脊柱控制使用标准控制器；控制周期由 launch 从 `controller_update_rate` 推导为硬件参数 `control_period`。Camera 与 LiDAR 由 `Ros2SensorAdapter` 从同一个 `RobotHardware` 经独立线程发布，不进入控制循环的标量状态接口。`mfr3duo_moveit`、`mfr3duo_nav` 和 `mfr3duo_robot` 当前只声明包边界及依赖；规划、导航与整机应用逻辑留待后续任务。

父仓库的 gitlink 固定两个 submodule 的准确提交；`.gitmodules` 的 `develop` 仅用于显式执行 `git submodule update --remote` 时选取上游分支。
