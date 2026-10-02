# mfr3duo_ros2

Mobile FR3 Duo 的 ROS 2 Humble 集成工作区（Ubuntu 22.04）。本仓库管理六个包及其构建顺序；`mfr3duo_description` 和 `mfr3duo_mujoco` 是固定提交的 Git submodule。`mfr3duo_hardware` 提供 MuJoCo ros2_control 插件、标准控制器配置和传感器桥；MoveIt、Nav 和 robot 包仍只提供包边界与依赖声明。

## 获取源码

```bash
git clone --recurse-submodules -b develop https://github.com/SiYueY/mfr3duo_ros2.git
cd mfr3duo_ros2
```

已有检出可运行 `git submodule update --init --recursive` 取得父仓库固定的提交。需要主动跟踪两个 submodule 的 `develop` 分支时，运行 `git submodule update --remote --recursive`，检查变更后提交父仓库更新的 gitlink。

## 依赖与构建

先安装 ROS 2 Humble、colcon 和六个包声明的 ROS 依赖，并构建安装包含 `LidarInfo::async_update` 的 `romujoco` 源码。仅满足 `romujoco >= 0.1.0` 的版本号不足以保证本工作区可编译。MuJoCo 依赖的完整准备方法见 [mfr3duo_mujoco 的构建说明](mfr3duo_mujoco/README.md)。例如，在工作区外执行：

```bash
git clone --recurse-submodules -b develop https://github.com/SiYueY/robot_mujoco.git
cd robot_mujoco/romujoco
./scripts/mujoco.sh build -j 2
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/romujoco-install" -DROMUJOCO_BUILD_TESTS=OFF
cmake --build build --parallel 2
cmake --install build
```

将 `romujoco` 的安装前缀加入 `CMAKE_PREFIX_PATH`。本工作区不要求安装 Pinocchio，因为 colcon 构建会关闭 MuJoCo teleop，独立构建的默认设置不变。

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$HOME/romujoco-install:${CMAKE_PREFIX_PATH}"
colcon list --topological-order
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

期望发现六个包；`mfr3duo_description` 先于 `mfr3duo_mujoco`，后者先于 `mfr3duo_hardware`。硬件包的链接测试检查 `mfr3duo_mujoco::mfr3duo_mujoco` 可被安装包消费者使用。更多边界与依赖关系见 [架构说明](docs/architecture.md)。

## 运行 MuJoCo 硬件

完成构建后，在同一 shell 中执行：

```bash
source install/setup.bash
ros2 launch mfr3duo_hardware mujoco_control.launch.py
```

默认打开 MuJoCo 仿真窗口，窗口展示 ros2_control 正在控制的同一个仿真实例。需要可用的桌面显示与 OpenGL 环境；无窗口运行可添加 `viewer_enabled:=false`。查看器由底层仿真管理，物理步进仍由 ros2_control 控制循环驱动。

启动文件从 description 生成 URDF，加载一个 `mfr3duo_hardware/Ros2ControlAdapter` 插件和关节状态、IMU、双臂轨迹、TMR 转向/驱动、脊柱控制器。可用 `ros2 control list_hardware_interfaces` 和 `ros2 control list_controllers` 检查接口与激活状态。控制频率只有 `controller_update_rate` 一个事实来源，默认 500 Hz；launch 把它换算成硬件参数 `control_period=1/controller_update_rate`，所以 1000 Hz 只需 `controller_update_rate:=1000`。这些参数指定目标周期，不保证墙钟硬实时。

升降接收绝对位置（单位 m，范围 0～0.85），例如：

```bash
ros2 topic pub --once /spine_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.24]}"
```

该控制器直接转发位置目标，不生成运动轨迹。底层升降位置伺服的阻尼已调整为 1000 N·s/m；`mfr3duo_mujoco.spine_response` 测试检查升降阶跃的超调和稳定性。修改底层参数并重新构建后，需要重启仿真才能生效。

Camera 和 LiDAR 从同一个 MuJoCo 实例发布到 `/sensors/<device>/image_raw`、`/sensors/<device>/camera_info` 与 `/sensors/lidar_front/scan`、`/sensors/lidar_rear/scan`。当前模拟相机输出为 320×180、25 Hz。`Ros2SensorAdapter` 在控制循环外发布消息。

`mfr3duo_hardware` 现在是两层：`libmfr3duo_hardware.so` 提供与 ROS 无关的 `RobotHardware` 整机 C++ API，`libmfr3duo_ros2_adapter.so` 提供 `Ros2ControlAdapter` 与 `Ros2SensorAdapter`。依赖方向固定为 `mfr3duo_ros2_adapter → mfr3duo_hardware → mfr3duo_mujoco`，Public Header 不包含任何 ROS 或 MuJoCo 类型。只安装 `robot_hardware.hpp`、`robot_types.hpp`、`visibility_control.hpp` 三个公共头文件。

纯 C++ 使用者可以直接持有 `RobotHardware` 而不启动 ROS；见 `test/robot_hardware_test.cpp`。

`RobotHardware` 的 motion snapshot 与生命周期状态由一把专用 mutex 保护，锁只覆盖状态检查与快照拷贝，控制路径的 `step`/`write_command` 与传感器读取都在锁外，因此控制线程与 `Ros2SensorAdapter` 线程可以并发。`test/robot_hardware_concurrency_test.cpp` 是这条契约的回归测试。`physics_period` 由 `initialize()` 从 backend 实测得出，不再在硬件层重复定义；代价是初始化时会多推进一个物理步。

注意：任何同时链接 MuJoCo 再加载 ROS 2 中间件的可执行文件（例如 `mfr3duo_hardware_interface_test` 与 `mfr3duo_ros2_adapter_cycle_benchmark`）都需要 `LD_PRELOAD=/lib/x86_64-linux-gnu/libtinyxml2.so.9`，否则 MuJoCo 导出的内置 tinyxml2 符号会拦截 Fast DDS 的系统 tinyxml2 调用。CMAKE 已把这些目标注册为带该环境变量的测试，手工运行时需要自行设置。

`mfr3duo_mujoco` 本次移除 `BaseCommand` / `BaseState`，改用 TMR 四主动关节及单独的被动状态 API；MFR3Duo 的 `JointControlMode` 仅保留 `Position=0`、`Velocity=1`、`Effort=2`。使用旧头文件编译的消费者必须更新源码并重新构建。`romujoco` 的通用 MobileBase 与 Hybrid 模式不变。
