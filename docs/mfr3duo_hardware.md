# mfr3duo_hardware RobotHardware 重构设计方案

## 1. 设计目标与总体原则

重构前 `mfr3duo_hardware` 主要以：

```cpp
class MujocoSystem final
    : public hardware_interface::SystemInterface;
```

作为核心实现。

> 当前实施状态：本重构已完成。`RobotHardware`、`Ros2ControlAdapter`、
> `Ros2SensorAdapter` 均已落地，`MujocoSystem` 与 `MujocoSensorBridge` 已删除。
> 下文出现的 `MujocoSystem` / `MujocoSensorBridge` 只用于描述重构前的状态与迁移步骤。

它同时承担：

```text
MuJoCo 生命周期
整机 Command / State
ros2_control 接口导出
机械臂 Command Mode Switching
IMU 状态转换
Camera / LiDAR ROS Topic 发布
```

这种设计能够快速完成 ROS 2 集成，但存在一个长期架构问题：

> MFR3Duo 的硬件能力实际上被 `ros2_control` 接口定义了。

这样会导致：

```text
直接 C++ 程序
机器人调试工具
Benchmark
Agent Runtime
未来真机 SDK
```

如果需要访问机器人，都必须绕过或者依赖 ROS 2。

本次重构将这一关系反转。

正式定义：

```text
mfr3duo_hardware::RobotHardware
```

为 MFR3Duo **唯一正式的整机 C++ Hardware API**。

ROS 2 不再是主接口，而是 `RobotHardware` 的适配层。

整体原则冻结为：

```text
RobotHardware
    = robot hardware semantics

Ros2ControlAdapter
    = RobotHardware → ros2_control

Ros2SensorAdapter
    = RobotHardware sensors → ROS messages/topics
```

核心依赖关系只能是：

```text
ROS 2
  ↓
RobotHardware
  ↓
backend
```

禁止反向依赖：

```text
RobotHardware
  ↓
ROS 2
```

因此 `RobotHardware` Public API 不得依赖：

```text
rclcpp
rclcpp_lifecycle
hardware_interface
controller_manager
pluginlib

sensor_msgs
geometry_msgs
trajectory_msgs
```

同时 Public API 也不直接暴露：

```text
mfr3duo_mujoco
MuJoCo mjModel
MuJoCo mjData
Simulation
```

这些都属于内部 backend 实现。

### 1.1 重构后的总体结构

```text
                    Application
               /       |        \
              /        |         \
             ▼         ▼          ▼
         Debugger   Benchmark    Agent
              \        |         /
               \       |        /
                ▼      ▼       ▼

                RobotHardware
             唯一正式 C++ API
                       │
                       ▼
             RobotHardware::Impl
                       │
                       ▼
          mfr3duo_mujoco::Simulation
                       │
                       ▼
                     MuJoCo
```

ROS 2 作为适配层：

```text
                     RobotHardware
                    /             \
                   /               \
                  ▼                 ▼
       Ros2ControlAdapter      Ros2SensorAdapter
               │                     │
               ▼                     ▼
          ros2_control          ROS messages
                                     │
                                     ▼
                                  Topics
```

因此：

```text
C++ API
```

是第一等接口。

而：

```text
hardware_interface::SystemInterface
ROS Topic
ROS Message
```

都是对第一等接口的外部表示。

### 1.2 本次重构不做什么

此次重构不引入：

```text
IHardware
HardwareBackend
BackendFactory
BackendManager
SensorManager
DeviceManager
Provider
Registry
```

也不创建复杂目录：

```text
core/
backend/
adapter/
manager/
device/
factory/
```

当前只有一个实际 backend：

```text
mfr3duo_mujoco
```

因此使用 PImpl 隔离实现已经足够。

等真实机器人 backend 真正开始开发，并出现明确的代码复用需求后，再决定是否抽象：

```text
Backend
├── MujocoBackend
└── RealRobotBackend
```

不能为了未来可能存在的需求提前设计一套 backend framework。

---

# 2. RobotHardware Public API 与整机数据模型

`RobotHardware` 是整个 `mfr3duo_hardware` 的核心。

建议 Public Header：

```text
include/
└── mfr3duo_hardware/
    ├── robot_hardware.hpp
    ├── robot_types.hpp
    └── visibility_control.hpp
```

其中：

```text
robot_hardware.hpp
```

只负责 `RobotHardware` 主类。

```text
robot_types.hpp
```

定义 `RobotHardware` 的全部 Public 数据类型：

```text
运动设备 Command / State
IMU / LiDAR / Camera 传感器数据
```

运动类型与传感器类型同属一个 Public 数据类型头文件，在文件内按段落区分，
不再拆分为独立的 `sensor_types.hpp`。

## 2.1 RobotHardware 主接口

建议接口：

```cpp
namespace mfr3duo_hardware {

class RobotHardware {
public:
  RobotHardware();
  ~RobotHardware();

  RobotHardware(RobotHardware&&) noexcept;
  RobotHardware& operator=(RobotHardware&&) noexcept;

  RobotHardware(const RobotHardware&) = delete;
  RobotHardware& operator=(const RobotHardware&) = delete;

  bool initialize(const RobotHardwareOptions& options);
  bool activate();
  bool deactivate();
  bool shutdown();

  bool update();

  bool write_command(const RobotCommand& command);

  bool read_state(RobotState& state) const;
  bool read_state(ImuState& state) const;

  bool read_state(
      Lidar id,
      LaserScan& scan) const;

  bool read_state(
      Camera id,
      CameraFrame& frame) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}
```

这是唯一正式整机接口。

典型使用方式：

```cpp
mfr3duo_hardware::RobotHardware robot;

mfr3duo_hardware::RobotHardwareOptions options;
options.control_period = std::chrono::milliseconds(2);

if (!robot.initialize(options)) {
  return false;
}

if (!robot.activate()) {
  return false;
}

mfr3duo_hardware::RobotCommand command;
mfr3duo_hardware::RobotState state;

robot.write_command(command);
robot.update();
robot.read_state(state);

robot.deactivate();
robot.shutdown();
```

外部程序不需要知道：

```text
MuJoCo
Simulation
ros2_control
controller_manager
```

## 2.2 为什么使用 update() 而不是 step()

当前 MuJoCo 底层使用：

```cpp
Simulation::step(N);
```

但：

```text
step
```

是 simulation 语义，不是 robot hardware 语义。

真实机器人不存在：

```cpp
robot.step();
```

如果把：

```cpp
RobotHardware::step()
```

冻结成 Public API，就会把 MuJoCo 的执行模型泄漏到未来真机接口。

因此 Public API 使用：

```cpp
bool update();
```

其定义是：

> 完成一次 RobotHardware 控制周期所需要的 backend 更新，并产生新的硬件状态快照。

MuJoCo 中：

```text
RobotHardware::update()
        ↓
Simulation::step(fixed_steps)
        ↓
读取 coherent RobotState
        ↓
更新 RobotHardware snapshot
```

未来真机可以实现成：

```text
RobotHardware::update()
        ↓
读取驱动 / SDK / EtherCAT
        ↓
更新 RobotHardware snapshot
```

因此调用者不感知 backend 类型。

## 2.3 RobotHardwareOptions

不应该向正式 API 暴露：

```text
simulation_steps_per_cycle
```

因为这同样属于 MuJoCo 实现细节。

Public API 使用：

```cpp
struct RobotHardwareOptions {
  std::chrono::nanoseconds control_period{
      std::chrono::milliseconds(2)};
};
```

例如：

```text
500 Hz  → 2 ms
1000 Hz → 1 ms
```

MuJoCo backend 初始化时根据：

```text
control_period
physics_period
```

计算：

```text
steps_per_update
```

例如：

```text
physics_period = 1 ms
control_period = 2 ms

steps_per_update = 2
```

要求：

```text
control_period % physics_period == 0
```

否则：

```text
initialize() == false
```

这个计算只在初始化时完成。

实现约定：

```text
physics_period ← backend
```

`physics_period` 不在 `mfr3duo_hardware` 里重复定义。`initialize()` 在
backend 初始化之后，用一次：

```text
time() → step(1) → time()
```

的差值实测物理步长，再据此计算 `steps_per_update`。这样 backend 始终是唯一
事实来源，也不会因为只改动 backend 而静默算错 step 数。`mfr3duo_mujoco`
不因此新增 Public API。

副作用：`initialize()` 会因此多推进一个物理步（1 ms）。`activate()` 仍然会重新
读取状态并提交 safe hold，所以不会产生跳变。

运行过程中禁止根据 wall-clock jitter 动态改变 step 数。

这样仍然保持当前仿真的确定性：

```text
500 Hz → 固定推进 2 个物理步
1000 Hz → 固定推进 1 个物理步
```

但不会把 simulation-specific 参数暴露给普通用户。

---

## 2.4 RobotCommand

整机只保留一个正式 command path：

```cpp
bool write_command(const RobotCommand&);
```

建议：

```cpp
struct RobotCommand {
  TmrCommand tmr;
  SpineCommand spine;

  ArmCommand left_arm;
  ArmCommand right_arm;

  GripperCommand left_gripper;
  GripperCommand right_gripper;
};
```

V1 不同时公开：

```cpp
write_command(const ArmCommand&);
write_command(const TmrCommand&);
write_command(const SpineCommand&);
write_command(const GripperCommand&);
```

避免形成多条设备级写命令路径。

正式语义：

```text
RobotCommand
    ↓
validate all
    ↓
prepare all
    ↓
atomic whole-robot commit
```

如果任意成员非法：

```text
RobotCommand rejected
```

不得出现：

```text
Left Arm command 已执行
Right Arm command 已执行
TMR command 校验失败
Spine 仍为旧值
```

这种 partial commit。

---

## 2.5 Arm

双机械臂继续支持：

```text
Position
Velocity
Effort
```

建议将控制模式提升到 `ArmCommand`，而不是每个 joint 单独保存：

```cpp
enum class JointControlMode : std::uint8_t {
  Position = 0,
  Velocity = 1,
  Effort = 2,
};

struct JointCommand {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct ArmCommand {
  JointControlMode mode{
      JointControlMode::Position};

  std::array<JointCommand, 7> joints{};
};
```

这样：

```text
Left  = Position
Right = Effort
```

可以自然表达。

而：

```text
left joint1 = Position
left joint2 = Velocity
```

在数据结构层面就不允许出现。

这比后续在 `write_command()` 内检查 7 个 joint mode 是否一致更可靠。

状态：

```cpp
struct JointState {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct ArmState {
  std::array<JointState, 7> joints{};
};
```

Hardware 层不提供：

```text
Cartesian pose
Cartesian velocity
Cartesian trajectory
IK / FK
redundancy resolution
PTP
whole-body control
```

这些仍由 controller / MoveIt / upper layer 负责。

---

## 2.6 Spine

正式 Command：

```cpp
struct SpineCommand {
  double position{0.0};
};
```

State：

```cpp
struct SpineState {
  double position{0.0};
  double velocity{0.0};
};
```

V1 不提供：

```text
velocity command
effort command
```

原因是 common hardware contract 应由真实设备能力决定，而不是 MuJoCo 能力决定。

高级行为：

```text
move_absolute
move_to_height
velocity profile
acceleration profile
halt
recovery
```

也不属于 `RobotHardware`。

---

## 2.7 TMR

正式 command：

```cpp
struct TmrCommand {
  double front_steering_position{0.0};
  double front_drive_velocity{0.0};

  double rear_steering_position{0.0};
  double rear_drive_velocity{0.0};
};
```

状态：

```cpp
struct TmrState {
  JointState front_steering;
  JointState front_drive;

  JointState rear_steering;
  JointState rear_drive;
};
```

Hardware API 不接受：

```text
vx
vy
wz
geometry_msgs/Twist
```

正确控制链：

```text
cmd_vel
   ↓
Swerve Controller
   ↓
Swerve IK
   ↓
TmrCommand
   ↓
RobotHardware
```

反馈：

```text
TmrState
   ↓
Swerve FK
   ↓
vx / vy / wz
   ↓
Odometry
```

因此：

```text
Swerve IK
Swerve FK
Odometry
cmd_vel timeout
speed limiting
TF publishing
```

均不属于 `RobotHardware`。

Passive TMR joint 也不进入真机 / 仿真的 common RobotState。

如果 MuJoCo 调试确实需要：

```text
rocker_arm
caster steering
caster wheel
```

可以继续作为 simulation diagnostic capability 保留在 `mfr3duo_mujoco`，但不进入 `RobotHardware` V1。

---

## 2.8 Gripper

保持现有物理语义：

```cpp
struct GripperCommand {
  double width{0.0};
  double velocity{0.0};
  double effort{0.0};
};
```

```cpp
struct GripperState {
  double width{0.0};
  double velocity{0.0};
  double effort{0.0};
  bool stalled{false};
};
```

Hardware 负责：

```text
width
velocity
effort
stalled
```

不负责：

```text
open()
close()
grasp()
homing
goal tolerance
grasp detection policy
ROS action lifecycle
```

这些由 Gripper Controller 负责。

---

## 2.9 RobotState

整机状态：

```cpp
struct RobotState {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};

  TmrState tmr;
  SpineState spine;

  ArmState left_arm;
  ArmState right_arm;

  GripperState left_gripper;
  GripperState right_gripper;
};
```

`RobotState` 定义为：

> 同一次 `RobotHardware::update()` 产生的 coherent whole-robot active motion snapshot。

不包含：

```text
IMU
LiDAR
Camera
TMR passive joints
simulation time
simulation step
```

主要原因是 Camera、LiDAR、IMU 的采样节奏与运动设备不一定一致。

特别是：

```text
simulation_time
simulation_step
```

属于仿真诊断，而不是机器人的 common hardware state。

以后如有需要，可以在 `mfr3duo_mujoco` 中保留：

```text
SimulationDiagnostics
```

但不进入 `RobotHardware::RobotState`。

---

# 3. 传感器 API 与时间模型

IMU、LiDAR 和 Camera 不应该首先被定义成 ROS Topic。

它们首先是：

```text
RobotHardware C++ data API
```

这些类型与运动设备的 Command / State 一起定义在同一个 Public 类型头文件：

```text
include/mfr3duo_hardware/robot_types.hpp
```

文件内按段落区分运动类型与传感器类型。

ROS Topic 只是其中一种输出方式。

## 3.1 IMU

定义：

```cpp
struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct ImuState {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};

  Quaternion orientation;
  Vector3 angular_velocity;
  Vector3 linear_acceleration;
};
```

读取：

```cpp
ImuState imu;

if (!robot.read_state(imu)) {
  ...
}
```

IMU 的 C++ API 是源接口。

ROS 2 可以随后转换：

```text
ImuState
   ↓
Ros2ControlAdapter
   ↓
10 × StateInterface<double>
   ↓
IMUSensorBroadcaster
   ↓
sensor_msgs/msg/Imu
```

因此继续使用标准 `IMUSensorBroadcaster` 没有问题。

关键区别在于：

> IMU 的本体接口是 `ImuState`，而不是 ROS Topic。

---

## 3.2 LiDAR

ID：

```cpp
enum class Lidar : std::uint8_t {
  Front,
  Rear,
};
```

数据：

```cpp
struct LaserScan {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};

  std::string frame_id;

  float angle_min{0.0F};
  float angle_max{0.0F};
  float angle_increment{0.0F};

  float time_increment{0.0F};
  float scan_time{0.0F};

  float range_min{0.0F};
  float range_max{0.0F};

  std::vector<float> ranges;
  std::vector<float> intensities;
};
```

调用：

```cpp
LaserScan scan;

robot.read_state(
    Lidar::Front,
    scan);
```

不把 LiDAR 放进：

```cpp
hardware_interface::StateInterface<double>
```

因为它是变长数据。

非法 ID（超出 `Lidar` 枚举范围的 `static_cast`）必须让 `read_state()` 返回
`false`，不得静默回退到 `Front` 或 `Rear`。

---

## 3.3 Camera

Camera ID：

```cpp
enum class Camera : std::uint8_t {
  FrontColor,
  FrontDepth,

  RearColor,
  RearDepth,

  LeftColor,
  LeftDepth,

  RightColor,
  RightDepth,

  LeftWristColor,
  LeftWristDepth,

  RightWristColor,
  RightWristDepth,

  HeadZedLeft,
  HeadZedRight,
};
```

图像类型使用 C++ 自有结构，例如：

```cpp
struct Image {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};

  std::string encoding;

  bool is_bigendian{false};

  std::vector<std::uint8_t> data;
};
```

CameraInfo：

```cpp
struct CameraInfo {
  std::uint32_t width{0};
  std::uint32_t height{0};

  std::string distortion_model;

  std::vector<double> d;

  std::array<double, 9> k{};
  std::array<double, 9> r{};
  std::array<double, 12> p{};

  std::uint32_t binning_x{0};
  std::uint32_t binning_y{0};
};
```

CameraFrame：

```cpp
struct CameraFrame {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};

  std::string frame_id;
  std::string optical_frame_id;

  Image image;
  CameraInfo camera_info;
};
```

`CameraFrame` 只保留一个 `image`。backend 对深度流把图像放在自己的 depth
image 中，因此实现约定为：

```text
Camera::*Depth  → backend depth image
其余 Camera ID  → backend colour image
```

两者都映射到同一个 `CameraFrame::image`。`camera_info` 按 backend 原值映射，
不在这里改写尺寸。

非法 ID（超出 `Camera` 枚举范围的 `static_cast`）必须让 `read_state()` 返回
`false`，不得静默回退到 `FrontColor`。

调用：

```cpp
CameraFrame frame;

robot.read_state(
    Camera::FrontColor,
    frame);
```

Public API 中不能出现：

```cpp
sensor_msgs::msg::Image
sensor_msgs::msg::CameraInfo
```

---

## 3.4 Timestamp

Public API 不使用：

```cpp
rclcpp::Time
```

统一使用：

```cpp
std::uint64_t timestamp_ns;
```

表示 backend 统一时间基准下的纳秒时间。

MuJoCo 当前使用：

```text
simulation monotonic time
```

未来真机应使用：

```text
hardware monotonic timestamp
```

或统一映射后的 monotonic robot timestamp。

ROS 时间转换属于：

```text
Ros2ControlAdapter
Ros2SensorAdapter
```

的职责。

即：

```text
RobotHardware timestamp
           ↓
ROS Adapter
           ↓
builtin_interfaces/Time
           ↓
Header.stamp
```

Public Hardware API 不依赖 ROS clock。

---

# 4. 生命周期、控制周期和线程模型

RobotHardware 必须拥有自己的生命周期，而不是把生命周期语义交给 ros2_control。

## 4.1 生命周期

冻结：

```text
Uninitialized
      │
 initialize()
      ▼
   Inactive
      │
  activate()
      ▼
    Active
      │
 deactivate()
      ▼
   Inactive
      │
 shutdown()
      ▼
Uninitialized
```

### initialize()

负责：

```text
创建 backend
初始化 MuJoCo Simulation
加载模型
检查 control_period
计算固定 update step 数
初始化传感器资源
创建内部状态 buffer
获取初始状态
```

成功后进入：

```text
Inactive
```

### activate()

进入 Active 前必须先获得有效状态。

随后构造 safe initial command。

Arm：

```text
position = current position
velocity = 0
effort   = 0
```

Spine：

```text
position = current position
```

TMR：

```text
front steering = current position
rear steering  = current position

front drive = 0
rear drive  = 0
```

Gripper：

```text
width    = current width
velocity = 0
effort   = 0
```

这样 controller 或直接 C++ 用户激活后不会产生突然跳变。

### deactivate()

必须先提交安全命令：

```text
arms:
    hold current position / zero velocity / zero effort

spine:
    hold position

TMR:
    hold steering
    zero drive

gripper:
    hold current width
```

随后停止接受正常运动命令。

### shutdown()

负责：

```text
处于 Active 时先 best-effort 提交安全停机命令
停止后台 sensor task
等待必要线程退出
关闭 Simulation
释放 backend resource
清理 internal state
```

安全停机命令失败也必须继续释放资源，并最终回到：

```text
Uninitialized
```

绝不把对象留在 `Active`。返回值为各步骤的综合结果。析构函数同样走这条路径，
不绕开 `RobotHardware` 自己的生命周期语义。

重复 `shutdown()` 不应该产生 undefined behavior。

---

## 4.2 update / write / read 顺序

推荐周期：

```text
write_command(previous/new command)
            │
            ▼
         update()
            │
            ▼
      backend progresses
            │
            ▼
    refresh RobotState
            │
            ▼
       read_state()
```

在 ros2_control 中实际会映射为：

```text
read()
    ↓
RobotHardware::update()
    ↓
RobotHardware::read_state()

controller_manager.update()
    ↓
controllers compute command

write()
    ↓
RobotHardware::write_command()
```

这意味着：

> 当前 `write()` 写入的命令在下一轮 `update()` 时被 backend 执行。

这与现有固定步进仿真语义一致。

---

## 4.3 Sensor 并发

控制路径：

```text
update()
write_command()
read_state(RobotState)
```

属于控制线程。

而：

```text
read_state(Camera, ...)
read_state(Lidar, ...)
```

可能来自 `Ros2SensorAdapter` 的独立线程。

因此 `RobotHardware` 必须支持：

```text
control update
      ||
sensor snapshot read
```

并发。

实现约定：

```text
motion snapshot + lifecycle
        ↓
   专用 mutex
```

`RobotState` 快照与生命周期状态由同一把专用 mutex 保护，锁只覆盖"检查状态"和
"拷贝快照"两段极短区间，绝不跨越 backend 调用（`step` / `write_command` /
传感器读取）。Camera 与 LiDAR 读取不经过这把锁，因此不会被控制周期串行化。

`Ros2SensorAdapter` 读 `RobotState`（用于时间基准）与控制线程 `update()` 写快照
之间的竞争，正是由这把锁消除的。

但不要求生命周期接口：

```text
initialize
activate
deactivate
shutdown
```

可以与 control / sensor API 并发调用。

调用者必须保证生命周期操作串行。

不建议在 `RobotHardware` 外层增加一个全局 mutex：

```text
update
camera
lidar
```

全部互斥。

应该继续复用当前 `mfr3duo_mujoco` 已经形成的：

```text
coherent snapshot
camera worker
lidar worker
independent mjData
```

机制。

目标是：

```text
Camera / LiDAR
```

不能阻塞主控制周期。

---

## 4.4 Error semantics

正式 API V1 可以继续使用：

```cpp
bool
```

而不是现在立即引入：

```text
expected<T,E>
Result<T>
ErrorCode framework
exception hierarchy
```

但要明确：

```text
false
```

代表该次操作未成功完成。

`write_command()` 返回 false 时：

```text
新的整机命令没有被部分提交。
```

`update()` 返回 false 时：

```text
不得把未完成更新产生的中间数据作为新 RobotState 发布。
```

`read_state()` 返回 false 时：

```text
调用方不得假设 output 已更新。
```

如果未来错误处理确实变复杂，再演进 Result 类型。

---

# 5. ROS 2 Adapter 设计

ROS 2 integration 统一使用：

```text
Adapter
```

命名。

不再使用：

```text
MujocoSystem
RosSensorBridge
```

最终：

```text
Ros2ControlAdapter
Ros2SensorAdapter
```

两者命名和职责完全对称。

---

## 5.1 Ros2ControlAdapter

定义：

```cpp
class Ros2ControlAdapter final
    : public hardware_interface::SystemInterface;
```

它虽然需要实现：

```cpp
hardware_interface::SystemInterface
```

但这个继承关系只是为了让：

```text
controller_manager
pluginlib
```

加载。

它不是第二套 Hardware API。

内部：

```cpp
class Ros2ControlAdapter final
    : public hardware_interface::SystemInterface {
public:
  ...

private:
  RobotHardware robot_;

  RobotState robot_state_;
  ImuState imu_state_;
  RobotCommand robot_command_;

  // ros2_control backing storage
  ...
};
```

它绝对不能直接持有：

```cpp
mfr3duo_mujoco::Simulation
```

也不能直接调用：

```cpp
Simulation::step()
Simulation::read_state()
Simulation::write_command()
```

所有机器人访问必须经过：

```cpp
RobotHardware
```

---

## 5.2 ROS lifecycle 映射

`on_init()`：

```text
解析 HardwareInfo
校验 joint / GPIO / sensor interface
读取 ROS adapter 配置
```

此时不创建或启动机器人。

ros2_control 侧的硬件参数冻结为：

```text
control_period
```

它是秒的十进制字符串，例如 `0.002`。旧的：

```text
simulation_steps_per_cycle
```

不再存在。launch 从：

```text
controller_update_rate
```

推导：

```text
control_period = 1 / controller_update_rate
```

因此 500 Hz 对应 `0.002`，1000 Hz 对应 `0.001`，控制频率只有一个事实来源。
不是 1 ms 整数倍、非正或无法解析的 `control_period` 依次在 `on_init()` 与
`on_configure()` 处失败。

`on_configure()`：

```text
RobotHardware::initialize()
```

`on_activate()`：

```text
RobotHardware::activate()
        ↓
read RobotState
        ↓
初始化 ros2_control command backing storage
```

`read()`：

```text
RobotHardware::update()
        ↓
read_state(RobotState)
        ↓
read_state(ImuState)
        ↓
copy to ros2_control state storage
```

`write()`：

```text
ros2_control command storage
        ↓
RobotCommand
        ↓
RobotHardware::write_command()
```

`on_deactivate()`：

```text
RobotHardware::deactivate()
```

`on_cleanup()`：

```text
RobotHardware::shutdown()
```

`on_shutdown()` / `on_error()`：

同样进入安全停止并释放资源。

---

## 5.3 Command Mode Switching

以下接口仍然只属于：

```text
Ros2ControlAdapter
```

```cpp
prepare_command_mode_switch()
perform_command_mode_switch()
```

因为它们处理的是：

```text
ros2_control resource claiming
```

而不是底层机器人生命周期。

例如 controller 要从：

```text
left_arm position
```

切到：

```text
left_arm effort
```

Ros2ControlAdapter 负责验证：

```text
7 个 left arm joint 是否一起切换
是否出现 mixed interface
stop/start resource 是否完整
```

通过后转换为：

```cpp
robot_command_.left_arm.mode =
    JointControlMode::Effort;
```

RobotHardware 本身不需要知道：

```text
start_interfaces
stop_interfaces
controller claiming
```

它只理解：

```text
ArmCommand.mode
```

这种机器人语义。

---

## 5.4 ros2_control Contract 保持不变

此次重构不改变当前已经完成的 ros2_control interface contract。

继续保持：

| Device | Command | State |
|---|---|---|
| Left FR3 ×7 | position / velocity / effort | position / velocity / effort |
| Right FR3 ×7 | position / velocity / effort | position / velocity / effort |
| Spine | position | position / velocity |
| TMR front steering | position | position / velocity |
| TMR front drive | velocity | position / velocity |
| TMR rear steering | position | position / velocity |
| TMR rear drive | velocity | position / velocity |
| Left Gripper | width / velocity / effort | width / velocity / effort / stalled |
| Right Gripper | width / velocity / effort | width / velocity / effort / stalled |
| IMU | — | orientation / angular velocity / acceleration |

仍然应满足：

```text
Command Interface = 53
State Interface   = 70
```

因此当前：

```text
JointTrajectoryController
ForwardCommandController
Spine controller
IMUSensorBroadcaster
```

配置原则上不需要因为此次架构重构而改变。

---

## 5.5 Ros2SensorAdapter

Camera / LiDAR 使用：

```cpp
class Ros2SensorAdapter;
```

它内部只持有：

```cpp
RobotHardware&
```

例如：

```cpp
class Ros2SensorAdapter {
public:
  explicit Ros2SensorAdapter(
      RobotHardware& robot);

  void start();
  void stop();

private:
  RobotHardware& robot_;
};
```

读取：

```cpp
robot_.read_state(
    Lidar::Front,
    scan);
```

以及：

```cpp
robot_.read_state(
    Camera::FrontColor,
    frame);
```

随后完成：

```text
LaserScan
      ↓
sensor_msgs::msg::LaserScan
```

以及：

```text
CameraFrame
      ↓
sensor_msgs::msg::Image
sensor_msgs::msg::CameraInfo
```

Ros2SensorAdapter 只负责：

```text
C++ type → ROS message
timestamp mapping
ROS QoS
Topic naming
publish
```

它不负责：

```text
sensor acquisition
MuJoCo rendering
LiDAR ray calculation
robot state
sensor business logic
```

这些都必须在 `RobotHardware` 或 backend 中完成。

---

## 5.6 两个 Adapter 必须共享同一个 RobotHardware

ROS 模式下：

```cpp
class Ros2ControlAdapter {
private:
  RobotHardware robot_;
  std::unique_ptr<Ros2SensorAdapter> sensor_adapter_;
};
```

创建：

```cpp
sensor_adapter_ =
    std::make_unique<Ros2SensorAdapter>(
        robot_);
```

因此：

```text
Ros2ControlAdapter
          │
          ▼
     RobotHardware
          ▲
          │
Ros2SensorAdapter
```

必须是同一实例。

绝不允许：

```text
Ros2ControlAdapter
    └── RobotHardware A

Ros2SensorAdapter
    └── RobotHardware B
```

否则 MuJoCo 下会启动两份仿真，真机下则可能形成两个独立硬件 session。

---

## 5.7 Plugin 命名

当前：

```text
mfr3duo_hardware/MujocoSystem
```

改为：

```text
mfr3duo_hardware/Ros2ControlAdapter
```

Plugin：

```cpp
PLUGINLIB_EXPORT_CLASS(
    mfr3duo_hardware::Ros2ControlAdapter,
    hardware_interface::SystemInterface)
```

XML：

```xml
<class
  name="mfr3duo_hardware/Ros2ControlAdapter"
  type="mfr3duo_hardware::Ros2ControlAdapter"
  base_class_type="hardware_interface::SystemInterface"/>
```

Xacro：

```xml
<hardware>
  <plugin>
    mfr3duo_hardware/Ros2ControlAdapter
  </plugin>
</hardware>
```

这里：

```text
Ros2ControlAdapter
```

虽然需要导出给 pluginlib，但不属于给普通开发者直接调用的 Public C++ API。

---

# 6. 代码组织、构建和依赖边界

本次重构继续保持目录简单。

建议最终结构：

```text
mfr3duo_hardware/
├── CMakeLists.txt
├── package.xml
├── mfr3duo_hardware.xml
│
├── include/
│   └── mfr3duo_hardware/
│       ├── robot_hardware.hpp
│       ├── robot_types.hpp
│       └── visibility_control.hpp
│
├── src/
│   ├── robot_hardware.cpp
│   │
│   ├── ros2_control_adapter.hpp
│   ├── ros2_control_adapter.cpp
│   │
│   ├── ros2_sensor_adapter.hpp
│   └── ros2_sensor_adapter.cpp
│
├── ros2_control/
│   ├── mfr3duo.ros2_control.xacro
│   └── mfr3duo_ros2_control_macros.xacro
│
├── config/
│   └── controllers.yaml
│
├── launch/
│   └── mujoco_control.launch.py
│
└── test/
    ├── robot_hardware_test.cpp
    ├── robot_hardware_concurrency_test.cpp
    ├── interface_test.cpp
    ├── runtime_integration.py
    ├── control_cycle_benchmark.cpp
    ├── ros2_control_adapter_cycle_benchmark.cpp
    ├── hardware_info.hpp
    ├── link_mujoco.cpp
    └── long_running_test.cpp
```

不要为了表示 adapter 再增加：

```text
src/adapter/
```

当前只有两个 adapter，没有必要增加目录层级。

---

## 6.1 RobotHardware 内部

`robot_hardware.cpp`：

```cpp
struct RobotHardware::Impl {
  mfr3duo_mujoco::Simulation simulation;

  RobotState state;
  ImuState imu_state;

  RobotCommand command;

  std::size_t steps_per_update{1};

  ...
};
```

Public Header 只有：

```cpp
struct Impl;
std::unique_ptr<Impl> impl_;
```

因此：

```text
mfr3duo_mujoco
```

不会泄露到 Public API。

---

## 6.2 mfr3duo_mujoco 与 Hardware 类型

`mfr3duo_mujoco` 可以继续有自己的：

```text
RobotCommand
RobotState
ImuState
LaserScan
CameraFrame
```

因为它仍然是独立的纯 C++ MuJoCo 模块。

但是 `RobotHardware` Public API 不应该直接：

```cpp
using RobotState =
    mfr3duo_mujoco::RobotState;
```

也不应：

```cpp
#include <mfr3duo_mujoco/simulation.hpp>
```

出现在 Public Header。

内部应该做明确映射：

```text
mfr3duo_hardware::RobotCommand
                  ↓
mfr3duo_mujoco::RobotCommand
```

状态反向：

```text
mfr3duo_mujoco::RobotState
                  ↓
mfr3duo_hardware::RobotState
```

原因很明确：

> `RobotHardware` 定义的是机器人 contract，`mfr3duo_mujoco` 定义的是 MuJoCo backend contract。

二者当前很相似，不代表它们应该成为同一个 public type。

否则以后接真机时：

```text
mfr3duo_hardware
```

仍然会被迫依赖 `mfr3duo_mujoco`。

---

## 6.3 CMake Target

建议至少拆两个 library。

### Core Hardware Library

```text
mfr3duo_hardware
```

生成：

```text
libmfr3duo_hardware.so
```

它包含：

```text
RobotHardware
RobotHardware public types
MuJoCo backend implementation
```

依赖：

```text
PRIVATE:
    mfr3duo_mujoco
```

不依赖：

```text
rclcpp
hardware_interface
pluginlib
sensor_msgs
```

普通 C++ 用户：

```cmake
target_link_libraries(
    my_application
    PRIVATE
    mfr3duo_hardware
)
```

即可。

### ROS 2 Adapter Library

```text
mfr3duo_ros2_adapter
```

生成：

```text
libmfr3duo_ros2_adapter.so
```

包含：

```text
Ros2ControlAdapter
Ros2SensorAdapter
```

依赖：

```text
mfr3duo_hardware

hardware_interface
pluginlib
rclcpp
rclcpp_lifecycle

sensor_msgs
...
```

这样依赖方向非常直观：

```text
mfr3duo_ros2_adapter
          ↓
mfr3duo_hardware
          ↓
mfr3duo_mujoco
```

而不是：

```text
mfr3duo_hardware
          ↓
ROS 2
```

---

## 6.4 Public Header 安装

只安装：

```text
robot_hardware.hpp
robot_types.hpp
visibility_control.hpp
```

`ros2_control_adapter.hpp` 和：

```text
ros2_sensor_adapter.hpp
```

可以留在：

```text
src/
```

仅供 package 自身构建。

它们不是 SDK 接口。

---

# 7. 测试、迁移步骤与最终冻结边界

这次重构虽然主要修改架构，但必须避免“换了一层包装后原有能力退化”。

因此测试应重新划分为：

```text
RobotHardware tests
ROS adapter tests
```

而不是所有行为都依赖 ROS integration test 验证。

## 7.1 RobotHardware 单元/集成测试

这是此次重构后最重要的测试层。

完全不启动 ROS。

至少覆盖：

```text
initialize
activate
deactivate
shutdown

invalid lifecycle call
reinitialize
safe activation
safe deactivation

shutdown while Active
    安全停机后回到 Uninitialized
```

并发契约：

```text
control thread: write_command + update
reader threads: read_state(RobotState)

同一 sequence 必须对应完全相同的快照
```

由 `robot_hardware_concurrency_test` 承担，防止快照保护被改回裸访问。

运动接口：

```text
Left Arm Position
Left Arm Velocity
Left Arm Effort

Right Arm Position
Right Arm Velocity
Right Arm Effort

Left / Right independent control mode

Spine Position

TMR front steering
TMR rear steering

TMR front drive
TMR rear drive

Left Gripper
Right Gripper
```

状态：

```text
RobotState sequence

RobotState timestamp

whole-robot snapshot coherence

Joint position
Joint velocity
Joint effort

Gripper stalled
```

传感器：

```text
IMU valid state

Front / Rear LiDAR

all Camera IDs

Camera dimensions
Camera calibration
Camera frame IDs
```

错误语义：

```text
invalid RobotCommand
      ↓
false
      ↓
no partial command update
```

同时验证：

```text
read_state() failure
```

不会向调用者发布半更新状态。

设备 ID 边界：

```text
read_state(static_cast<Lidar>(255), scan) == false
read_state(static_cast<Camera>(255), frame) == false
最后一个合法 ID 仍然可读
```

---

## 7.2 ROS Adapter 测试

ROS 测试主要验证映射。

### Ros2ControlAdapter

继续测试：

```text
pluginlib load

53 command interfaces
70 state interfaces

interface names
interface order independence

left arm mode switching
right arm mode switching

partial arm switch rejected
mixed arm switch rejected
```

Controller integration：

```text
JointStateBroadcaster

Left JointTrajectoryController
Right JointTrajectoryController

TMR Steering ForwardCommandController
TMR Drive ForwardCommandController

Spine Position Controller

IMUSensorBroadcaster
```

### Ros2SensorAdapter

验证：

```text
Front LiDAR → sensor_msgs/LaserScan

Front Color → sensor_msgs/Image

Depth Camera → Image + CameraInfo

frame_id
optical frame
timestamp
camera intrinsics
QoS
```

重点不再重复验证：

```text
MuJoCo Camera 能否渲染
LiDAR 数据是否正确生成
```

因为这些应已经由：

```text
RobotHardware tests
```

覆盖。

ROS Adapter test 只关心：

```text
C++ data → ROS representation
```

是否正确。

---

## 7.3 性能验收

控制路径：

```text
RobotHardware::write_command()
RobotHardware::update()
RobotHardware::read_state(RobotState)
```

继续作为正式 benchmark 对象。

要求：

```text
无 ROS publisher
无 ROS callback
无文件 IO
无 Camera rendering
无 LiDAR ray casting
尽量无周期性 heap allocation
```

当前已有：

```text
500 Hz
1000 Hz
```

benchmark 可以迁移为直接测量：

```text
RobotHardware
```

而不是必须经过 `Ros2ControlAdapter`。

然后另外增加轻量 ROS integration benchmark，检查 Adapter 没有产生明显额外开销。

Camera / LiDAR：

```text
data generation
ROS serialization
ROS publishing
```

仍然不得进入主控制线程。

---

## 7.4 推荐迁移顺序

为了降低一次性重构风险，按照以下顺序实施。

### 第一阶段：建立 RobotHardware

新增：

```text
robot_hardware.hpp
robot_types.hpp
robot_hardware.cpp
```

内部直接复用现有：

```text
mfr3duo_mujoco::Simulation
```

先把现有：

```text
MujocoSystem → Simulation
```

中的 hardware semantics 搬入：

```text
RobotHardware
```

完成后要求 standalone 测试可以：

```text
arm
spine
TMR
gripper
IMU
Camera
LiDAR
```

全部直接通过 C++ API 工作。

此阶段原 `MujocoSystem` 暂时可以继续存在。

### 第二阶段：重构 ros2_control

将：

```text
MujocoSystem
```

改为：

```text
Ros2ControlAdapter
```

删除其中：

```text
mfr3duo_mujoco::Simulation simulation_;
```

替换为：

```cpp
RobotHardware robot_;
```

所有：

```text
simulation_.step()
simulation_.read_state()
simulation_.write_command()
```

替换为：

```text
robot_.update()
robot_.read_state()
robot_.write_command()
```

然后恢复全部：

```text
53 command
70 state
controller integration
```

测试。

### 第三阶段：重构 Sensor

将当前：

```text
MujocoSensorBridge
```

改为：

```text
Ros2SensorAdapter
```

从：

```text
MujocoSensorBridge
        ↓
Simulation
```

改为：

```text
Ros2SensorAdapter
        ↓
RobotHardware
```

删除 ROS Sensor Adapter 对：

```text
mfr3duo_mujoco
```

的直接依赖。

### 第四阶段：清理 Public Boundary

确认：

```text
include/mfr3duo_hardware/
```

中没有：

```text
ROS headers
MuJoCo headers
mfr3duo_mujoco headers
```

并确认普通纯 C++ 示例只需：

```cpp
#include <mfr3duo_hardware/robot_hardware.hpp>
```

即可完成机器人访问。

### 第五阶段：删除旧边界

全部迁移完成后删除：

```text
MujocoSystem
MujocoSensorBridge
```

名称和旧 plugin name。

不保留两套长期兼容 API。

如果当前项目尚未正式发布 V1，可以直接 clean break。

---

## 7.5 最终冻结的模块边界

重构结束后，整个模块只需要理解以下关系：

```text
                  mfr3duo_hardware
                         │
                         ▼
                  RobotHardware
             唯一正式 C++ Hardware API
                         │
              ┌──────────┴──────────┐
              │                     │
              ▼                     ▼
       motion hardware          sensor hardware
              │                     │
              │                     │
       RobotCommand              ImuState
       RobotState                LaserScan
                                 CameraFrame
              │                     │
              └──────────┬──────────┘
                         │
                         ▼
               RobotHardware::Impl
                         │
                         ▼
              mfr3duo_mujoco
```

ROS：

```text
                       RobotHardware
                      /             \
                     /               \
                    ▼                 ▼
         Ros2ControlAdapter      Ros2SensorAdapter
                  │                     │
                  ▼                     ▼
             ros2_control          ROS Topics
```

正式 Public API 是：

```text
RobotHardware
RobotHardwareOptions

RobotCommand
RobotState

JointControlMode
JointCommand
JointState

ArmCommand
ArmState

TmrCommand
TmrState

SpineCommand
SpineState

GripperCommand
GripperState

ImuState

Lidar
LaserScan

Camera
Image
CameraInfo
CameraFrame
```

内部实现：

```text
RobotHardware::Impl
Ros2ControlAdapter
Ros2SensorAdapter
```

不是正式 Hardware API：

```text
hardware_interface::SystemInterface
controller_manager
ROS Topic
ROS Message

Simulation
MuJoCo step
simulation time
simulation step

Swerve IK / FK
cmd_vel
Odometry

trajectory generation
Cartesian control
MoveIt
Nav2
whole-body coordination
```

最终可以用一句话冻结整个设计：

```text
RobotHardware 是 MFR3Duo 唯一正式的整机 C++ Hardware API。

Ros2ControlAdapter 将 RobotHardware 适配为 ros2_control。

Ros2SensorAdapter 将 RobotHardware 的传感器数据适配为 ROS 2 消息。

ROS 2 与 MuJoCo 都不能反向定义 RobotHardware 的公共接口。
```

这使得当前：

```text
MuJoCo
```

只是第一个 backend。

未来底层替换成：

```text
真实 FR3
TMR SDK
Spine Driver
Gripper Driver
真实 Camera / LiDAR / IMU
```

时，上层仍然可以继续面对同一套：

```cpp
mfr3duo_hardware::RobotHardware
```

而无需重新设计机器人接口。
