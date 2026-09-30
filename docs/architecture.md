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

`mfr3duo_hardware` 通过一个 `SystemInterface` 插件管理 MuJoCo 仿真，导出双臂、脊柱、TMR 主动关节、夹爪和 IMU 的 ros2_control 接口。关节状态、IMU、双臂轨迹、TMR 转向/驱动和脊柱控制使用标准控制器。Camera 与 LiDAR 从同一仿真实例经独立线程发布，不进入控制循环的标量状态接口。`mfr3duo_moveit`、`mfr3duo_nav` 和 `mfr3duo_robot` 当前只声明包边界及依赖；规划、导航与整机应用逻辑留待后续任务。

父仓库的 gitlink 固定两个 submodule 的准确提交；`.gitmodules` 的 `develop` 仅用于显式执行 `git submodule update --remote` 时选取上游分支。
