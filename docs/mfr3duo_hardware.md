# MFR3Duo Hardware 完整设计方案

> 本文是设计与验收规格；各阶段的实际完成情况以代码和测试结果为准。本方案明确变更此前冻结的 `mfr3duo_mujoco` Public API；这些破坏性变更仅发生在 MFR3Duo 层，`romujoco` 保留通用能力。

## 当前实施状态

| 阶段 | 状态 | 验证依据 |
|---|---|---|
| Phase 1：TMR 与 Public API | 已实现 | motor 模型、TMR 主动/被动读取、原子整机命令、standalone teleop 与安装包消费者测试 |
| Phase 2：`MujocoSystem` | 已实现 | ros2_control 插件加载、接口检查、生命周期启动/停止及模式切换测试 |
| Phase 3：Controllers | 已实现 | 双臂轨迹、脊柱位置、TMR 转向/驱动命令的运行集成测试 |
| Phase 4：Sensors 与性能 | 已实现，非硬实时 | LiDAR/Camera/IMU 发布与 60 秒仿真测试通过；充分预热后控制线程在完整传感器基准中未发生 C++ `new`；500/1000 Hz 已测量，偶发墙钟超时见性能基线 |

上表是当前工作区的实现状态；向其他工作区分发前，还需把两个 submodule 的变更提交并更新父仓库 gitlink。

## 1. 设计目标与边界

`mfr3duo_hardware` 定义为：

> Mobile FR3 Duo 面向 ROS 2 的基础硬件抽象层，只暴露机器人能够直接执行或感知的基础物理能力。

核心原则：

```text
Hardware exposes physical primitives,
not robot behaviors.
```

也就是：

```text
mfr3duo_hardware
    ├── actuator command
    ├── actuator state
    ├── device state
    └── raw sensor state
```

不承担：

```text
IK / FK
trajectory planning
trajectory generation
Cartesian planning
Swerve IK / FK
odometry integration
navigation
grasp behavior
whole-body coordination
MoveIt
Nav2
Agent logic
```

完整分层：

```text
Application / Agent
        │
        ▼
  mfr3duo_robot
        │
   ┌────┴─────┐
   ▼          ▼
mfr3duo_moveit   mfr3duo_nav
   │              │
   └──────┬───────┘
          ▼
   ROS Controllers
          │
          ▼
   mfr3duo_hardware
          │
     ┌────┴────┐
     ▼         ▼
 Real Robot   MuJoCo
               │
               ▼
       mfr3duo_mujoco
               │
               ▼
           romujoco
```

Franka ROS 2 是：

```text
engineering reference
```

而不是：

```text
compatibility target
```

采用 Franka 好的设计：

```text
✓ ros2_control SystemInterface
✓ pluginlib
✓ hardware lifecycle
✓ command mode switching
✓ joint-level actuator interface
✓ simulation / real common contract
✓ fixed backing storage
✓ controller 与 hardware 分层
```

不采用：

```text
✗ robot_state pointer-as-double
✗ robot_model pointer interface
✗ Franka-specific elbow representation
✗ 为兼容 Franka 暴露 franka_msgs
✗ 把 Cartesian planning 塞入 hardware
✗ 把设备历史 REST 架构当成统一机器人设计
```

---

# 2. MFR3Duo Hardware Contract

## 2.1 FR3 双机械臂

两个 FR3 都作为标准 7-DOF joint actuator group。

关节：

```text
left_fr3v2_1_joint1 ... joint7
right_fr3v2_1_joint1 ... joint7
```

每个关节：

```text
Command:
    position
    velocity
    effort

State:
    position
    velocity
    effort
```

这是 V1 唯一正式的机械臂 hardware command contract。

### 不进入 Hardware

以下能力全部上移：

```text
Cartesian pose
Cartesian velocity
Cartesian trajectory
IK
Elbow / redundancy resolution
PTP motion
null-space control
whole-body coordination
```

例如：

```text
Pose target
    ↓
MoveIt / Servo / Cartesian Controller
    ↓
Joint command
    ↓
mfr3duo_hardware
```

而不是：

```text
Pose
    ↓
mfr3duo_hardware
```

### TCP wrench（候选能力，暂不进入 V1）

TCP wrench 可能是硬件状态，但当前没有冻结其数据来源、坐标系、符号方向、时间戳和有效性语义。真机需要确认使用 FR3 外力估计还是独立传感器；MuJoCo 需要明确使用 site force/torque sensor、接触力汇总还是其他估计方式。这些来源的物理含义不能直接视为相同。

上述语义和两侧数据源通过独立验证前，不导出 `left_tcp` / `right_tcp` 正式接口。力控制、阻抗控制和接触任务仍属于上层 controller。

当前 `mfr3duo_description/mjcf/mfr3duo.xml` 只有关节执行器力等相关 sensor，没有左右 TCP 的 site force / torque sensor；关节执行器力也不能直接当成 TCP wrench。MuJoCo 侧要取得可用数据，仍需新增并标定测量来源，再与真机来源核对坐标系、方向和时间戳。因此 Phase 4 的验证结论应保持“候选、未冻结”，不能以现有 joint effort 冒充完成的 wrench 接口。

### Hybrid mode

重构前，`mfr3duo_mujoco::JointControlMode` 包含：

```text
Hybrid
Position
Velocity
Effort
```

重构后的 MFR3Duo Public API 已移除 `Hybrid`，并显式定义枚举数值：

```cpp
enum class JointControlMode : std::uint8_t {
    Position = 0,
    Velocity = 1,
    Effort = 2,
};
```

`JointCommand` 移除仅用于 Hybrid 的 `stiffness` / `damping` 字段；保留 `mode`、`position`、`velocity`、`effort` 及其现有默认值。`JointState::mode` 继续保留，并从底层主动关节状态正确转换。`SpineCommand` / `SpineState` 继续是 joint alias；hardware V1 只导出 Spine position command，不能因为仿真层的类型范围更宽而额外导出速度或力命令。

这是有意的 Public API / ABI 破坏性变更：旧值为 `Hybrid=0, Position=1, Velocity=2, Effort=3`，新值不可按整数直接转发到底层。必须显式转换、更新所有消费者和测试，并在发布说明中记录迁移；已编译的旧消费者需要重新构建。

`romujoco::JointMode::Hybrid` 及其 `MobileBaseCommand` / `MobileBaseState` 作为通用 runtime 能力保留，不因 MFR3Duo 的契约变化而修改。阻抗或全身控制由上层 controller 计算输出到 effort interface。

---

## 2.2 TMR Mobile Base

这里是本次最重要的重构。

重构前：

```cpp
BaseCommand {
    linear_x;
    linear_y;
    angular_z;
};
```

直接删除。

同时：

```cpp
BaseState {
    Pose;
    Twist;
};
```

不再作为正式 hardware state。

### TMR 的真实执行器结构

四个主动关节：

```text
tmrv0_2_joint_0
    front steering

tmrv0_2_joint_1
    front drive

tmrv0_2_joint_2
    rear steering

tmrv0_2_joint_3
    rear drive
```

这与当前 `mfr3duo_description` 中的真实机械模型一致。

正式 contract：

```text
tmrv0_2_joint_0:
    Command: position
    State:   position, velocity

tmrv0_2_joint_1:
    Command: velocity
    State:   position, velocity

tmrv0_2_joint_2:
    Command: position
    State:   position, velocity

tmrv0_2_joint_3:
    Command: velocity
    State:   position, velocity
```

不暴露：

```text
vx
vy
wz
```

作为 hardware command。

### Swerve 控制属于上层

正确链路：

```text
geometry_msgs/Twist
        │
        ▼
MFR3Duo Swerve Controller
        │
        ├── front steering position
        ├── front drive velocity
        ├── rear steering position
        └── rear drive velocity
        │
        ▼
mfr3duo_hardware
```

反馈：

```text
TMR joint states
        │
        ▼
Swerve FK
        │
        ▼
vx / vy / wz
        │
        ▼
Odometry
```

所以：

```text
Swerve IK
Swerve FK
cmd_vel timeout
speed limiter
odometry
TF publishing
```

全部不属于 hardware。

Franka 的 TMR 也采用 steering-position / drive-velocity 的底层接口，并把 SwerveDriveController 独立在 hardware 之外；这里采用的是它的分层思想，而不是其完整 API。

### Passive joints

MFR3Duo 模型还有：

```text
rocker_arm_joint

caster_front_left_steering_joint
caster_front_left_joint

caster_rear_right_steering_joint
caster_rear_right_joint
```

这些没有 command，只存在：

```text
State:
    position
    velocity
```

MuJoCo 可以提供这些状态。

真机如果没有传感器，则不存在对应 state source。

因此定义为 `simulation auxiliary state`，经独立 `read_state(TmrPassiveState&)` 读取；它们不进入 `TmrState`、`RobotState` 或上层控制器依赖的 common contract。

---

## 2.3 Spine

Spine 的物理模型是：

```text
franka_spine_vertical_joint
```

一个 prismatic joint：

```text
range:
    0.0 → 0.85 m
```

Hardware 层应该把它看成物理执行轴，而不是：

```text
MoveAbsolute behavior
REST device
```

但这里需要尊重真实硬件能力。

V1 建议正式 contract：

```text
franka_spine_vertical_joint

Command:
    position

State:
    position
    velocity
```

暂时不导出：

```text
velocity command
effort command
```

原因不是 MuJoCo 做不到，而是当前 Franka Spine 真机公开接口主要是 profile position motion；在没有确认真实设备支持 cyclic velocity / effort command 前，不应该让 simulation capability 反过来扩大 hardware contract。

如果以后真实驱动确认支持：

```text
continuous velocity control
```

再增加：

```text
velocity command
```

### 高级 Spine 行为

这些全部属于上层：

```text
move_absolute()
move_to_height()
trajectory
acceleration profile
deceleration profile
halt policy
automatic recovery
```

而 hardware 可以额外提供：

```text
enabled
fault
ready
limit
```

等真实设备状态。

---

## 2.4 Gripper

Gripper 与普通 revolute/prismatic joint 不同。

其真实控制 primitive 更接近：

```text
opening width
velocity
force
```

因此保留当前 semantic direction。

每个 Gripper：

```text
Command:
    width
    velocity
    effort

State:
    width
    velocity
    effort
    stalled
```

推荐 ros2_control 表达为 GPIO/custom interfaces：

```text
left_gripper/width
left_gripper/velocity
left_gripper/effort

right_gripper/width
right_gripper/velocity
right_gripper/effort
```

状态：

```text
left_gripper/width
left_gripper/velocity
left_gripper/effort
left_gripper/stalled
```

而 URDF 中：

```text
finger_joint1
finger_joint2
```

仍然用于：

```text
robot model
RViz
MoveIt
collision
```

上层 Gripper Controller 负责：

```text
width
   ↓
finger joint representation
```

以及：

```text
open()
close()
move()
grasp()
```

这些行为。

Hardware 不负责：

```text
homing policy
grasp detection policy
goal tolerance
action lifecycle
```

---

## 2.5 Sensors

传感器同样只提供 raw physical data。

### IMU

IMU 是固定大小数据，可以使用标准 ros2_control sensor interfaces：

```text
imu/orientation.x
imu/orientation.y
imu/orientation.z
imu/orientation.w

imu/angular_velocity.x
imu/angular_velocity.y
imu/angular_velocity.z

imu/linear_acceleration.x
imu/linear_acceleration.y
imu/linear_acceleration.z
```

然后由标准 broadcaster 发布：

```text
sensor_msgs/msg/Imu
```

### LiDAR

LiDAR 是变长数据：

```text
sensor_msgs/msg/LaserScan
```

不应该塞入：

```text
StateInterface<double>
```

MuJoCo sensor bridge 读取：

```cpp
mfr3duo_mujoco::LaserScan
```

直接转换为 ROS `LaserScan`。

### Camera

同理：

```text
sensor_msgs/msg/Image
sensor_msgs/msg/CameraInfo
```

Camera 不进入 ros2_control realtime scalar interface。

LiDAR / Camera bridge 必须读取 `MujocoSystem` 持有的**同一** `Simulation` 实例，不能自行初始化第二个 MuJoCo 仿真。桥接采集和 ROS 发布在控制循环之外的工作线程执行；插件停用时先停止并等待该线程，再关闭仿真。消息保留设备 `frame_id`、图像编码、相机标定和扫描参数，并按样本 `sequence` 去重。仿真时间与 ROS `Header.stamp` 的映射须明确，不能直接把底层单调时钟时间戳当成 ROS 时间。

当前桥接在每轮采集时以 ROS 当前时间减去仿真当前时间计算偏移，再将样本的仿真纳秒时间戳加到该偏移上；同一轮的各传感器因此共用时间基准。这样仿真慢于墙钟时，消息仍接近实际发布时间，但跨采集轮的时间差会受仿真与墙钟速率差影响。该映射不提供 `/clock`；需要严格模拟时间的应用应切换到完整的 `/clock` 方案。

Camera 消息使用模型的 optical frame。桥接沿用底层计算的 K/P 内参；对底层未填的 depth-only `CameraInfo` 宽高，从实际 depth image 补齐，并为无畸变的 MuJoCo pinhole 相机填充零畸变和单位阵 R。运行集成测试检查 RGB 图像、depth CameraInfo 尺寸与内参、LiDAR 扫描和 IMU 消息。

Hardware 只负责：

```text
frame acquisition
timestamp
transport
```

不负责：

```text
detection
depth fusion
tracking
VIO
```

---

# 3. mfr3duo_mujoco 同步重构

`mfr3duo_mujoco` 必须和新的底层 contract 对齐。`romujoco` 继续保留通用 `Joint`、`MobileBase`、`Gripper` 和传感器能力；本节只修改 MFR3Duo 模型与适配层。

## 3.1 删除 Base API

删除：

```cpp
struct BaseCommand;
struct BaseState;
```

以及：

```cpp
bool write_command(const BaseCommand&);
bool read_state(BaseState&) const;
```

`RobotCommand`：

```cpp
BaseCommand base;
```

也删除。

`RobotState`：

```cpp
BaseState base;
```

同样删除。

不保留 deprecated alias，也不增加兼容 shim。这一 clean break 会影响现有 standalone teleop、示例、安装包消费者和 base 相关测试；Phase 1 必须同步迁移并复测，不能只修改头文件。现有 teleop 的底盘按键仍产生 `vx / vy / wz` 意图，迁移时须在 teleop 或上层控制器转换为 TMR steering / drive 目标，不再直接写 `BaseCommand`。

---

## 3.2 新增 TMR actuator API

建议：

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
    double timestamp{0.0};

    JointState front_steering;
    JointState front_drive;

    JointState rear_steering;
    JointState rear_drive;
};
```

被动状态使用不带控制模式的独立类型：

```cpp
struct PassiveJointState {
    double position{0.0};
    double velocity{0.0};
};

struct TmrPassiveState {
    double timestamp{0.0};
    PassiveJointState rocker_arm;
    PassiveJointState front_caster_steering;
    PassiveJointState front_caster_wheel;
    PassiveJointState rear_caster_steering;
    PassiveJointState rear_caster_wheel;
};
```

底层被动关节的 `romujoco::JointMode::None` 不得转换成 MFR3Duo 的 `Position`。`TmrState` 只含四个主动关节；`TmrPassiveState` 是独立的仿真辅助读取结果。

### Public Simulation API

调整为：

```cpp
bool write_command(const TmrCommand& command);

bool read_state(TmrState& state) const;
bool read_state(TmrPassiveState& state) const;
```

整机：

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

状态：

```cpp
struct RobotState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    double simulation_time{0.0};
    std::uint64_t step{0};

    TmrState tmr;
    SpineState spine;
    ArmState left_arm;
    ArmState right_arm;
    GripperState left_gripper;
    GripperState right_gripper;
};
```

仍然保持：

> coherent whole-robot active-motion snapshot

`RobotState` 从同一份底层 `romujoco::RobotState` 提取主动运动设备；不逐设备拼接，也不复制被动 TMR、IMU、Camera 或 LiDAR 数据。`RobotCommand` 一次转换并提交一份完整底层命令，设备级写入仍是局部更新。ROS 2 对外使用分散 scalar interface，不改变仿真层的整机快照语义。

---

## 3.3 不再使用 `romujoco::SwerveMobileBase`

重构前：

```text
mfr3duo_mujoco
    ↓
romujoco::SwerveMobileBase
    ↓
PlanarTwist
```

重构后：

```text
mfr3duo_mujoco
    ↓
4 × romujoco::Joint
```

即：

```text
tmrv0_2_joint_0
    JointMode::Position

tmrv0_2_joint_1
    JointMode::Velocity

tmrv0_2_joint_2
    JointMode::Position

tmrv0_2_joint_3
    JointMode::Velocity
```

重构前的 MJCF 使用 MuJoCo 原生 position / velocity servo：

```text
tmrv0_2_joint_0_position
tmrv0_2_joint_1_velocity
tmrv0_2_joint_2_position
tmrv0_2_joint_3_velocity
```

`romujoco::Joint` 自身按目标模式计算控制量，并把结果写入其 actuator；不能未经验证就把上述原生 servo 直接挂到 `Joint` 上。Phase 1 必须在 `mfr3duo.xml` 与 `franka_tmr.xml` 中把这四个 actuator 改为基础 motor / effort actuator，同时更新 `mfr3duo_mujoco` 的 component 配置、名称映射和相关测试：

```text
front steering / rear steering: allowed_modes = {Position}
front drive / rear drive:       allowed_modes = {Velocity}
```

需验证 motor 控制量、限幅、方向和停止保持行为，再固定增益。`romujoco::Joint`、`romujoco::MobileBase` 的通用抽象不改。

Generic：

```cpp
romujoco::MobileBase
```

可以继续存在供其他机器人使用。

只是：

```text
MFR3Duo 不再使用它。
```

这是比修改 generic `romujoco` 更合理的边界。

---

## 3.4 TMR component IDs 重构

重构前：

```text
0      spine
1..7   left arm
8..14  right arm
15..19 passive TMR
```

建议重新整理为语义明确的 ID：

```cpp
namespace joint {

inline constexpr JointId kSpine = 0;

inline constexpr std::array<JointId, 7>
    kLeftArm{1,2,3,4,5,6,7};

inline constexpr std::array<JointId, 7>
    kRightArm{8,9,10,11,12,13,14};

namespace tmr {

inline constexpr JointId kFrontSteering = 15;
inline constexpr JointId kFrontDrive = 16;
inline constexpr JointId kRearSteering = 17;
inline constexpr JointId kRearDrive = 18;

inline constexpr JointId kFrontCasterSteering = 19;
inline constexpr JointId kFrontCasterWheel = 20;
inline constexpr JointId kRockerArm = 21;
inline constexpr JointId kRearCasterSteering = 22;
inline constexpr JointId kRearCasterWheel = 23;

}

}
```

不要继续保留：

```cpp
namespace mobile_base
```

因为 MFR3Duo 不再注册 MobileBase component；`romujoco` 的 MobileBase 类型仍然存在。

---

## 3.5 Ground Truth

不建议现在增加：

```cpp
BaseGroundTruth
```

作为正式 Public API。

因为当前设计目标是：

```text
robot hardware semantics
```

不是：

```text
simulation debug API
```

以后确实需要：

```text
perfect pose
perfect velocity
contact truth
```

再单独建立明确的：

```text
simulation diagnostics
```

API。

不要混入 `RobotState`。

---

# 4. mfr3duo_hardware 实现设计

## 4.1 MuJoCo 使用一个 SystemInterface

MuJoCo 中整个机器人共享：

```text
mjModel
mjData
physics step
```

因此 V1 使用：

```cpp
class MujocoSystem final
    : public hardware_interface::SystemInterface;
```

一个 hardware plugin。

它管理：

```text
14 arm joints
1 spine joint
4 TMR active joints
2 grippers
IMU
```

Camera / LiDAR 通过辅助 bridge 发布。

真机未来不要求必须也是一个 SystemInterface。

例如以后完全可以是：

```text
LeftArmSystem
RightArmSystem
TmrSystem
SpineSystem
```

只要它们导出的 **hardware contract 相同**。

所以：

> common contract 不等于 common implementation topology。

---

## 4.2 ros2_control Xacro

建议：

```text
mfr3duo_hardware/
└── ros2_control/
    ├── mfr3duo.ros2_control.xacro
    └── mfr3duo_ros2_control_macros.xacro
```

Macro：

```text
arm_joint
spine_joint
tmr_steering_joint
tmr_drive_joint
gripper
imu
```

不要复制 robot links/joints。

机械模型继续全部来自：

```text
mfr3duo_description
```

Hardware Xacro 只描述：

```text
command interfaces
state interfaces
hardware plugin
hardware parameters
```

---

## 4.3 建议的 package 结构

```text
mfr3duo_hardware/
├── CMakeLists.txt
├── package.xml
├── mfr3duo_hardware.xml
│
├── include/
│   └── mfr3duo_hardware/
│       ├── mujoco_system.hpp
│       ├── interface_names.hpp
│       └── visibility_control.hpp
│
├── src/
│   ├── mujoco_system.cpp
│   └── mujoco_sensor_bridge.cpp
│
├── ros2_control/
│   ├── mfr3duo.ros2_control.xacro
│   └── mfr3duo_ros2_control_macros.xacro
│
└── test/
    ├── interface_test.cpp
    ├── lifecycle_test.cpp
    ├── mode_switch_test.cpp
    ├── command_test.cpp
    └── simulation_test.cpp
```

不要增加：

```text
backend/
adapter/
factory/
manager/
device/
mapping/
```

除非实现后确实出现重复职责。

---

# 5. Runtime 与控制时序

## 5.1 Lifecycle

```text
on_init()
    │
    ├── parse HardwareInfo
    ├── validate joint names
    └── validate interface contract

on_configure()
    │
    └── Simulation::initialize()

on_activate()
    │
    ├── read initial state
    ├── initialize every RobotCommand member from current state
    └── zero velocity / effort command

read()
    │
    ├── Simulation::step(N)
    ├── read RobotState
    └── copy to ros2_control backing storage

controller_manager.update()
    │
    ▼

write()
    │
    ├── build RobotCommand
    └── Simulation::write_command()

on_deactivate()
    │
    └── safe command

on_cleanup()
    │
    └── Simulation::shutdown()

on_error() / on_shutdown()
    │
    ├── stop sensor bridge
    └── Simulation::shutdown()
```

MuJoCo 使用：

```cpp
Simulation::step()
```

而不是：

```cpp
Simulation::start()
```

固定 step 数使仿真推进量可重复，并使命令在下一次 `read()` 步进中生效；这不等于保证墙钟实时性。`read()` 步进或快照读取失败、`write()` 整机提交被拒绝时，接口应返回错误并按生命周期错误路径处理，不能发布部分命令或沿用未定义状态。

---

## 5.2 控制周期

当前 MJCF：

```text
physics timestep = 1 ms
```

即：

```text
1000 Hz physics
```

Hardware parameter：

```text
simulation_steps_per_cycle
```

例如：

```text
controller = 1000 Hz
steps_per_cycle = 1

controller = 500 Hz
steps_per_cycle = 2
```

不要根据每周期 wall-clock jitter 动态计算 step 数。

---

## 5.3 Activate 时安全初始化

### Arms

```text
position command = current position
velocity command = 0
effort command   = 0
```

### TMR

```text
front steering command = current steering
rear steering command  = current steering

front drive velocity = 0
rear drive velocity  = 0
```

### Spine

```text
position command = current position
```

### Gripper

```text
width command = current width
velocity = 0
effort = 0
```

防止 controller activate 后突然跳变。首次 `write()` 前必须初始化完整 `RobotCommand`，因为整机写入会提交所有成员，包括保持默认值的成员。停用、模式切换失败和重新激活时也要定义安全命令与状态恢复；这些行为需要 lifecycle 测试覆盖。

---

## 5.4 Arm command mode switching

左右 Arm 独立：

```text
Left:
    Position / Velocity / Effort

Right:
    Position / Velocity / Effort
```

允许：

```text
Left  = Position
Right = Effort
```

但不允许一个 Arm 内：

```text
joint1 = position
joint2 = velocity
```

因此：

```cpp
prepare_command_mode_switch()
```

必须验证整组 7 joints。

TMR 不需要 mode switching：

```text
steering = fixed Position
drive    = fixed Velocity
```

Spine V1：

```text
fixed Position
```

Gripper：

```text
fixed semantic interfaces
```

---

# 6. 开发顺序与验收

建议拆成四阶段，但每个阶段都必须完整可运行。

### Phase 1 — 重构 `mfr3duo_mujoco` 底盘

完成：

```text
删除 BaseCommand / BaseState

删除 MFR3Duo 对 romujoco::MobileBase 的使用

将 MFR3Duo MJCF 的 4 个 TMR 原生 servo 改为 motor，并配置 4 个 TMR active Joint 的固定 allowed_modes

新增 TmrCommand / TmrState
新增独立 TmrPassiveState 与读取 API

移除 MFR3Duo JointControlMode::Hybrid 和 JointCommand 的 stiffness / damping；显式重映射 P/V/E 数值，保留 JointState::mode

修改 RobotCommand / RobotState，保留 sequence、timestamp、simulation_time、step
```

并保证 standalone test 全部通过。

验收：

```text
front steering position command works
rear steering position command works

front drive velocity command works
rear drive velocity command works

motor 控制方向、位置 / 速度限幅、零速停车和转向保持均符合预期
stopped / fixed-step 后，RobotState 中全部主动设备状态与同一步的设备级读取一致
all passive joint states readable through the separate API
passive None mode is never reported as Position
whole-robot command is submitted once; rejected commands cause no partial update
新枚举的数值与默认 Position 通过静态断言和运行测试验证
standalone teleop 的 Twist 意图已转为 TMR 关节目标；示例、测试及安装包消费者已迁移并通过
```

---

### Phase 2 — `MujocoSystem`

实现：

```text
SystemInterface
pluginlib
lifecycle
arm P/V/E
spine position
TMR interfaces
gripper interfaces
IMU state
```

验证：

```bash
ros2 control list_hardware_interfaces
```

接口必须精确符合设计。

---

### Phase 3 — Controllers 集成验证

此阶段不是把 controller 写进 hardware，而是验证 hardware 足够通用。

至少使用：

```text
JointStateBroadcaster

JointTrajectoryController
    left arm
    right arm

ForwardCommandController
    TMR steering / drive test

Spine position controller
```

验证：

```text
controller
    ↓
ros2_control
    ↓
mfr3duo_hardware
    ↓
mfr3duo_mujoco
```

链路完整。

之后再在上层实现：

```text
Swerve controller
Gripper controller
```

---

### Phase 4 — Sensors 与性能收敛

增加：

```text
LiDAR bridge
Camera bridge

TCP wrench 仅做数据来源和语义验证；不加入 V1 正式接口

500 / 1000 Hz benchmark
allocation audit
long-running test
```

实时路径：

```text
read()
write()
```

要求：

```text
no repeated heap allocation
no blocking ROS calls
no publisher work
no file I/O
```

验收时须测量完整调用链，包括 `mfr3duo_mujoco` 与 `romujoco`。底层命令和状态缓冲复用未被读者持有的快照；读者长期持有所有预分配快照时，缓冲区仍会增长以保持已发布快照不可变。Camera 渲染和 LiDAR 射线计算在各自工作线程执行；物理线程提交的是同一时刻的 MuJoCo 状态副本，LiDAR 工作线程使用独立 `mjData`。相机批次在前一批结果消费后才重新提交，避免控制线程处理被覆盖的渲染结果。基准测试分别记录完整周期耗时、控制线程 C++ `new` 和所有线程 C++ `new`；这些计数不覆盖 C `malloc`、GPU 或驱动分配，也不代替 ROS 调度延迟测量。

### 当前性能基线（2026-09-30，本机 headless 测量）

构建测试目标后，在已经加载 ROS 2 与 colcon 工作区环境的终端运行构建目录中的 `mfr3duo_hardware/mfr3duo_control_cycle_benchmark`；`--no-sensors`、`--no-camera`、`--no-lidar`、`--no-imu` 可分别关闭对应传感器以定位耗时。每组预热 1000 次并测量 1000 次 `write_command(RobotCommand) → step(N) → read_state(RobotState) → read_state(ImuState)`；关闭 IMU 时跳过 IMU 读取。结果如下，单位为微秒；超时表示单次耗时超过目标周期，不代表 ROS 调度周期。全传感器数据是三次独立进程测量的范围：

| 配置 | 平均 | p99 | 最大 | 超时次数 / 1000 |
|---|---:|---:|---:|---:|
| 1000 Hz，Camera/LiDAR/IMU 开 | 234–243 | 278–385 | 423–628 | 0、0、0 |
| 500 Hz，Camera/LiDAR/IMU 开 | 458–486 | 531–1430 | 671–2403 | 0、0、1 |
| 1000 Hz，传感器关 | 232 | 275 | 361 | 0 |
| 500 Hz，传感器关 | 455 | 508 | 574 | 0 |

同一分配审计中，1000 次连续整机写入、1000 次连续运动状态读取，以及充分预热后的 1000 次全传感器完整控制周期，在控制线程上均记录为 0 次 C++ `operator new`。临时 glibc `malloc/calloc/realloc` 拦截诊断在一次全传感器进程中也记录为控制线程 0 次调用；诊断代码未加入产品或基准目标。相机/LiDAR 工作线程仍需为图像和扫描数据分配：上述三个全传感器 1000/500 Hz 组的所有线程合计约 352–381/702–746 次 C++ `new`；关闭全部传感器时为 0。该结果不覆盖 GPU、驱动或其他分配器。三个全传感器进程中有一次 500 Hz 周期超过 2 ms；随后五次独立进程测量在 500/1000 Hz 均为 0 次超时、控制线程 C++ `new` 为 0。当前不能承诺无超时或硬实时。

分项测量中，关闭 Camera、保留 LiDAR/IMU 时，1000/500 Hz 组平均约 233/451 µs，均无超时，控制线程 C++ `new` 为 0；关闭 LiDAR、保留 Camera/IMU 时约 235/460 µs，也均无超时、控制线程 C++ `new` 为 0。异步 LiDAR 扫描使用复制的物理状态和独立 `mjData`，扫描时间戳取该状态的仿真时间；消费者可能在下一次传感器周期才收到已完成的扫描。

长时间仿真测试 `mfr3duo_hardware_long_running_test` 在启用 Camera、LiDAR、IMU 后推进 60,000 个 1 ms 物理步（60 秒仿真时间），每 1000 步检查整机快照序列、步数及 LiDAR/Camera 非空样本；本次构建用时约 15 秒，通过。该测试证明此配置在上述时长内持续运行，不证明硬实时或无限时长稳定性。

---

## 最终 V1 Hardware Contract

最终可以把 `mfr3duo_hardware` V1 冻结成非常简单的一张表：

| 设备 | Command | State |
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
| LiDAR | — | LaserScan |
| Camera | — | Image / CameraInfo |

TCP wrench 是候选能力，待数据来源、坐标系、符号方向、时间戳和有效性语义验证后另行设计。被动 TMR 关节状态仅为仿真辅助数据，不属于上表的真机 / 仿真共同契约。

明确不属于 V1 Hardware Contract：

```text
Twist
BaseCommand(vx, vy, wz)
Cartesian arm pose
Cartesian arm velocity
IK
Swerve IK/FK
Odometry
trajectory
MoveAbsolute
grasp behavior
PTP
whole-body coordination
RobotModel pointer
Franka elbow interface
```

这套设计最大的价值不是“比 Franka 接口少”，而是把边界重新放正确：

```text
hardware
    = physical primitive

controller
    = control law / kinematics

planner
    = planning

robot
    = application semantics
```

这样以后无论底层换成 MuJoCo、Franka 官方驱动还是自己的 `robohardware`，上层结构都不会被某个 backend 的历史设计绑死。
