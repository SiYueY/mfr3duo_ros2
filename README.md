# mfr3duo_ros2

Mobile FR3 Duo 的 ROS 2 Humble 集成工作区（Ubuntu 22.04）。本仓库管理八个包及其构建顺序；`mfr3duo_description` 和 `mfr3duo_mujoco` 是固定提交的 Git submodule。`mfr3duo_hardware` 提供 MuJoCo ros2_control 插件和传感器桥；`mfr3duo_control` 管理控制器配置、launch 和 TMR 命令控制器；MoveIt 包提供联合规划与 MoveGroup facade；Nav 包提供实测里程计配套的全向导航配置和 Navigator；夹爪消息采用参考 Franka 的 mfr3duo_msgs，物体与工具观察使用标准 PoseStamped；robot 提供 Robot/Task 编排、整机 readiness 与真实物理 Pick/Place。

## 获取源码

```bash
git clone --recurse-submodules -b develop https://github.com/SiYueY/mfr3duo_ros2.git
cd mfr3duo_ros2
```

已有检出可运行 `git submodule update --init --recursive` 取得父仓库固定的提交。需要主动跟踪两个 submodule 的 `develop` 分支时，运行 `git submodule update --remote --recursive`，检查变更后提交父仓库更新的 gitlink。

## 依赖与构建

先安装 ROS 2 Humble、colcon 和各包声明的 ROS 依赖，并构建安装包含 `LidarInfo::async_update`、body pose snapshot 与可选 wheel damping/friction compensation 的 `romujoco` 源码。仅满足 `romujoco >= 0.1.0` 的版本号不足以保证本工作区可编译。MuJoCo 依赖的完整准备方法见 [mfr3duo_mujoco 的构建说明](mfr3duo_mujoco/README.md)。本轮验证的外部 SDK 基线为 `robot_mujoco` 提交 `7481a8cca26d9d8c169676756442fa0d39c73383`；两个模型/后端子模块由本仓库 gitlink 固定。

例如，在工作区外执行：

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

期望发现八个包；`mfr3duo_description` 先于 `mfr3duo_mujoco`，后者先于 `mfr3duo_hardware`。硬件包的链接测试检查 `mfr3duo_mujoco::mfr3duo_mujoco` 可被安装包消费者使用。更多边界与依赖关系见 [架构说明](docs/architecture.md)。

## 运行 MuJoCo 硬件

完成构建后，在同一 shell 中执行：

```bash
source install/setup.bash
ros2 launch mfr3duo_control control.launch.py
```

默认打开 MuJoCo 仿真窗口，窗口展示 ros2_control 正在控制的同一个仿真实例。需要可用的桌面显示与 OpenGL 环境；无窗口运行可添加 `viewer_enabled:=false`。查看器由底层仿真管理，物理步进仍由 ros2_control 控制循环驱动。

启动文件从 description 生成 URDF，加载一个 `mfr3duo_hardware/Ros2ControlAdapter` 插件和关节状态、IMU、双臂轨迹、TMR body-twist、单关节 spine 轨迹控制器。可用 `ros2 control list_hardware_interfaces` 和 `ros2 control list_controllers` 检查接口与激活状态。控制频率只有 `controller_update_rate` 一个事实来源，默认 500 Hz；launch 把它换算成硬件参数 `control_period=1/controller_update_rate`，所以 1000 Hz 只需 `controller_update_rate:=1000`。这些参数指定目标周期，不保证墙钟硬实时。

升降使用单关节 `FollowJointTrajectory` action，绝对位置单位 m、范围 0～0.85。例如：

```bash
ros2 action send_goal /spine_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [franka_spine_vertical_joint], points: [{positions: [0.24], time_from_start: {sec: 4}}]}}"
```

TMR 接收平面 body twist；需要持续发布以维持运动，停止发布后 0.5 s watchdog 将驱动目标归零：

```bash
ros2 topic pub -r 20 /tmr_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}"
```

TMR 的几何、安装方向、速度/加速度和转向限制位于
`mfr3duo_control/config/controllers.yaml`。Phase 1A 已通过回归；后端相机快照池的控制线程分配已修复，详见
[Phase 1A 实施报告](docs/phase1a-report.md)。Phase 1B / Gate 1 已通过：夹爪 action、真实 finger joint 状态投影和 Control facade 已验证，详见 [Gate 1 报告](docs/phase1b-report.md)。Phase 2A 已通过 37 项工作区测试，见 [Phase 2A 报告](docs/phase2a-report.md)；Phase 2B / Gate 2 已通过 39 项工作区测试，见 [Gate 2 报告](docs/phase2b-report.md)；Phase 3 的公开 MoveGroup、只读 Plan 和 Cartesian 执行已通过 43 项工作区测试，见 [Phase 3 报告](docs/phase3-report.md)。Phase 4 的实测里程计、全向 Nav2 与 Navigator 已通过 51 项工作区测试，见 [Phase 4 报告](docs/phase4-report.md) 和 [导航说明](mfr3duo_nav/README.md)。Phase 5 / Gate 3 已通过同实例真实抓取、释放和物理状态恢复，见 [Phase 5 报告](docs/phase5-report.md) 和 [物理抓取运行说明](mfr3duo_robot/README.md)。

Camera 和 LiDAR 从同一个 MuJoCo 实例发布到 `/sensors/<device>/image_raw`、`/sensors/<device>/camera_info` 与 `/sensors/lidar_front/scan`、`/sensors/lidar_rear/scan`。当前模拟相机输出为 320×180、25 Hz。`Ros2SensorAdapter` 在控制循环外发布消息。

`mfr3duo_hardware` 现在是两层：`libmfr3duo_hardware.so` 提供与 ROS 无关的 `RobotHardware` 整机 C++ API，`libmfr3duo_ros2_adapter.so` 提供 `Ros2ControlAdapter` 与 `Ros2SensorAdapter`。依赖方向固定为 `mfr3duo_ros2_adapter → mfr3duo_hardware → mfr3duo_mujoco`，Public Header 不包含任何 ROS 或 MuJoCo 类型。只安装 `robot_hardware.hpp`、`robot_types.hpp`、`visibility_control.hpp` 三个公共头文件。

纯 C++ 使用者可以直接持有 `RobotHardware` 而不启动 ROS；见 `test/robot_hardware_test.cpp`。

`RobotHardware` 的 motion snapshot 与生命周期状态由一把专用 mutex 保护，锁只覆盖状态检查与快照拷贝，控制路径的 `step`/`write_command` 与传感器读取都在锁外，因此控制线程与 `Ros2SensorAdapter` 线程可以并发。`test/robot_hardware_concurrency_test.cpp` 是这条契约的回归测试。`physics_period` 由 `initialize()` 从 backend 实测得出，不再在硬件层重复定义；代价是初始化时会多推进一个物理步。

注意：任何同时链接 MuJoCo 再加载 ROS 2 中间件的可执行文件（例如 `mfr3duo_hardware_interface_test` 与 `mfr3duo_ros2_adapter_cycle_benchmark`）都需要 `LD_PRELOAD=/lib/x86_64-linux-gnu/libtinyxml2.so.9`，否则 MuJoCo 导出的内置 tinyxml2 符号会拦截 Fast DDS 的系统 tinyxml2 调用。CMAKE 已把这些目标注册为带该环境变量的测试，手工运行时需要自行设置。

`mfr3duo_mujoco` 本次移除 `BaseCommand` / `BaseState`，改用 TMR 四主动关节及单独的被动状态 API；MFR3Duo 的 `JointControlMode` 仅保留 `Position=0`、`Velocity=1`、`Effort=2`。使用旧头文件编译的消费者必须更新源码并重新构建。`romujoco` 的通用 MobileBase 与 Hybrid 模式不变。

左右夹爪 action 使用单指位置 0～0.04 m，wire max_effort=0 请求非零默认 effort：

```bash
ros2 action send_goal /left_gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 0.0}}"
```

Control 公共 C++ API 位于 `mfr3duo_control/control.hpp`，链接导出 target `mfr3duo_control::mfr3duo_control`。
应用在其它线程运行 Executor，再调用 `initialize(timeout)`；命令成功表示已收到成功的终止结果。
facade 先完成清理，再停止并 join 应用 Executor，最后调用 `rclcpp::shutdown()`。
状态 getter 必须检查 Result，区分未收到与过期；取消 terminal 未确认时同资源拒绝新命令。

## MoveIt 单组规划

```bash
ros2 launch mfr3duo_moveit moveit.launch.py viewer_enabled:=false
```

该入口默认只包含一次 Control bringup；Control 已运行时传入 `start_control:=false`。
使用 `run_probe:=true` 验证 LeftArm、RightArm 和 Spine 的真实规划/执行。
七个精确 SRDF group 的存在不表示所有联合 pose 目标已受支持，具体边界见
[MoveIt 包说明](mfr3duo_moveit/README.md)。执行使用原版官方插件和默认入口；应用独占执行通道，停止由官方 stop 事件及应用层终止确认完成，见 [迁移报告](docs/official-execution-report.md)。

JointStateBroadcaster 只发布 21 个真实受控关节的 position/velocity，GPIO 留在硬件状态接口。
五个被动底盘关节由现有 SensorAdapter 经独立传感器 API 发布到同一 `/joint_states`，
供完整 TF/MoveIt state 使用，不增加 ros2_control 接口或新的仿真实例。

## 整机任务演示

```bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-robot-logs
ros2 launch mfr3duo_robot robot.launch.py run_demo:=true
```

默认打开同实例 MuJoCo viewer；无窗口可加 `viewer_enabled:=false`。演示串行执行
NavigateTask → PickTask → PlaceTask，实际验证接触、抬升、稳定持物与释放，最后输出
`ROBOT_DEMO_PASS`。演示已区分转场与精确抓放速度，实测任务约 46–49 秒（不含初始化）。
速度配置见 `mfr3duo_robot/config/robot.yaml`；成功标记包含 `task_seconds`。
启动前几秒出现等待 odom/map TF 的 INFO
属于 readiness 等待；启动尚未完成就按 Ctrl+C，会中断整个 bringup。演示初始化被中断时
输出 `ROBOT_DEMO_INTERRUPTED`，真实任务失败输出 `ROBOT_DEMO_FAIL`。
只启动整机供自己的应用使用时保留默认 `run_demo:=false`。
Robot 的初始化、任务所有权、取消与恢复使用方式见 [Robot 包说明](mfr3duo_robot/README.md)，
实施与验证结果见 [Phase 6 报告](docs/phase6-report.md)。

夹爪设备接口位于 `mfr3duo_msgs`，参考 Franka 官方 Jazzy 定义；不依赖 franka_msgs。
自定义抓取观察消息包已移除，物体感知使用标准 PoseStamped。见
[接口迁移报告](docs/msgs-migration-report.md)。

## C++ 函数命名

项目自有成员函数与自由函数统一使用小写下划线命名，例如 `is_ready()`、`move_gripper()`、`set_target_pose()`。框架接口的 override 和第三方 API 保留上游名称。`.clang-tidy` 中的 `readability-identifier-naming` 用于检查该约定，`.clang-format` 控制排版。

本轮公共 API 已统一重命名；外部 C++ 调用方需同步更新名称并重新编译，不保留驼峰兼容入口。
