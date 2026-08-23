# `mfr3duo_mujoco` 设计说明

> **所属仓库**：`mfr3duo_ros2`  
> **ROS 2 包名**：`mfr3duo_mujoco`  
> **目标平台**：Ubuntu 22.04 / ROS 2 Humble / C++17  
> **底层仿真库**：`robot_mujoco/mujoco_simulation`  
> **模型来源**：`mfr3duo_description`  
> **设计定位**：Mobile FR3 Duo 的 MuJoCo 仿真语义适配层。

---

# 1. 背景、目标与范围

## 1.1 背景

Mobile FR3 Duo 是一个由多种子系统组成的复合机器人，包括：

- TMR 全向移动底盘；
- Franka Spine 升降机构；
- 左右两套 FR3 七自由度机械臂；
- 左右末端执行器；
- IMU；
- 底盘视觉传感器；
- 腕部视觉传感器；
- 头部视觉传感器；
- LiDAR；
- 其他后续可扩展传感器。

项目已经将通用 MuJoCo 仿真能力放在：

    robot_mujoco
    └── mujoco_simulation

中。

`mujoco_simulation` 的职责是提供与机器人类型无关的 MuJoCo runtime，包括：

    model loading
    simulation lifecycle
    physics scheduler
    Joint
    IMU
    Camera
    LiDAR
    command buffer
    state buffer
    camera renderer
    viewer
    contact state
    reset / keyframe
    realtime throttling

它不应该知道：

    Mobile FR3 Duo
    TMR
    Franka FR3
    Franka Spine
    Franka Hand
    Swerve
    ROS 2
    ros2_control
    MoveIt
    Nav2
    WBC

等机器人或上层框架语义。

另一方面，如果直接让：

    mfr3duo_hardware

使用：

    mujoco_simulation

的通用：

    JointId
    CameraId
    ImuId
    ComponentId

访问机器人，将会导致大量 Mobile FR3 Duo 特有的：

    joint name
    component ID
    actuator name
    camera name
    sensor name
    command mode
    state mapping

进入 ROS 2 hardware 层。

这会使：

    mfr3duo_hardware

同时承担：

    ros2_control integration
    Mobile FR3 Duo model interpretation
    MuJoCo component registration
    MuJoCo ID mapping
    simulator configuration
    simulator lifecycle

等过多职责。

因此需要增加：

    mfr3duo_mujoco

作为：

> **Mobile FR3 Duo 与通用 `mujoco_simulation` 之间的机器人语义适配层。**

---

## 1.2 模块定位

整个系统的推荐分层为：

    ┌─────────────────────────────────────────────┐
    │                ROS 2 Ecosystem              │
    │                                             │
    │   MoveIt 2       Nav2       WBC / Control   │
    └───────────────────┬─────────────────────────┘
                        │
                        ▼
    ┌─────────────────────────────────────────────┐
    │             mfr3duo_hardware                │
    │                                             │
    │       ros2_control SystemInterface          │
    │       state / command interfaces            │
    │       lifecycle / mode switching            │
    └───────────────────┬─────────────────────────┘
                        │
                        │ native C++
                        ▼
    ┌─────────────────────────────────────────────┐
    │              mfr3duo_mujoco                 │
    │                                             │
    │    Mobile FR3 Duo simulation semantics      │
    │    model contract                           │
    │    semantic state / command                 │
    │    component registration / mapping         │
    └───────────────────┬─────────────────────────┘
                        │
                        │ native C++
                        ▼
    ┌─────────────────────────────────────────────┐
    │        robot_mujoco / mujoco_simulation     │
    │                                             │
    │       generic MuJoCo simulation runtime     │
    └───────────────────┬─────────────────────────┘
                        │
                        ▼
                    MuJoCo

模型资源独立存在于：

    mfr3duo_description
            │
            ▼
       MJCF / assets

其中：

    mfr3duo_description
    → 描述机器人是什么

    mujoco_simulation
    → 提供通用 MuJoCo 仿真能力

    mfr3duo_mujoco
    → 解释 Mobile FR3 Duo 如何映射到通用仿真能力

    mfr3duo_hardware
    → 将仿真机器人接入 ROS 2 / ros2_control

这是整个设计最重要的职责边界。

---

## 1.3 核心目标

`mfr3duo_mujoco` 需要完成：

1. 定义 Mobile FR3 Duo 的 MuJoCo 模型契约；
2. 将机器人语义转换为 `mujoco_simulation::SimulationConfig`；
3. 注册 TMR、Spine、双 FR3、末端执行器和传感器；
4. 将底层通用 `RobotState` 转换为 Mobile FR3 Duo 语义状态；
5. 将 Mobile FR3 Duo 语义命令转换为底层 `RobotCommand`；
6. 对模型名称、Joint、Actuator、Sensor 等契约进行严格验证；
7. 对外提供稳定、机器人语义明确的 C++ API；
8. 隐藏底层 Component ID、Joint ID 和 MuJoCo 实现细节；
9. 复用 `mujoco_simulation` 已有线程、Scheduler、Buffer、Viewer 和 Renderer；
10. 支持后续 `mfr3duo_hardware` 无需理解 MuJoCo 模型内部结构即可完成 ros2_control 接入。

---

## 1.4 非目标

`mfr3duo_mujoco` 不负责：

    ROS 2 Node
    rclcpp
    Topic
    Service
    Action
    ros2_control SystemInterface
    controller_interface
    MoveIt
    Nav2
    TF publication
    sensor_msgs publication
    controller_manager
    WBC algorithm
    Swerve IK
    trajectory generation
    motion planning

同时不负责：

    MuJoCo scheduler
    physics thread
    command buffer implementation
    state buffer implementation
    camera rendering thread
    viewer implementation
    MJCF parser
    generic Joint controller implementation

这些能力已经属于其他模块。

---

# 2. 架构与依赖关系

## 2.1 ROS 2 包属性

`mfr3duo_mujoco` 在工程组织上是：

> **ROS 2 ament package。**

但：

> **其核心 C++ library 必须保持 ROS runtime independent。**

因此允许：

    ament_cmake
    package.xml
    mfr3duo_description package dependency

但核心 library 不允许依赖：

    rclcpp
    rclcpp_lifecycle
    hardware_interface
    controller_interface
    pluginlib
    sensor_msgs
    geometry_msgs
    nav_msgs
    trajectory_msgs
    MoveIt
    Nav2

也不创建：

    rclcpp::Node

---

## 2.2 依赖关系

推荐依赖：

    mfr3duo_mujoco
        │
        ├── mujoco_simulation
        │
        └── mfr3duo_description
             └── installed MJCF resources

其中：

    mujoco_simulation

作为普通 CMake library 依赖：

    find_package(
        mujoco_simulation
        CONFIG
        REQUIRED)

`mfr3duo_description` 不需要向 `mfr3duo_mujoco` 暴露 C++ API。

其职责只是安装：

    share/mfr3duo_description/
    ├── mjcf/
    ├── urdf/
    └── ...

模型绝对路径由上层解析后传入：

    SimulationConfig::model_path

---

## 2.3 不依赖 `ament_index_cpp`

`mfr3duo_mujoco` 核心 library 不主动执行：

    get_package_share_directory()

也不依赖：

    ament_index_cpp

原因是：

> 模型资源定位属于部署环境问题，不属于机器人仿真核心语义。

推荐由：

    mfr3duo_hardware
    mfr3duo_bringup
    application

解析：

    mfr3duo_description

安装路径，然后调用：

    mfr3duo_mujoco::Simulation::initialize()

传入完整模型路径。

这样：

    mfr3duo_mujoco

即使在：

    非 ROS 2 C++ 程序
    unit test
    standalone application

中仍然可以使用。

---

## 2.4 不复制模型文件

`mfr3duo_mujoco` 中禁止出现：

    models/
    meshes/
    assets/
    mobile_fr3_duo.xml

模型只能存在于：

    mfr3duo_description

因此：

    mfr3duo_description
    =
    canonical model source

禁止形成：

    mfr3duo_description/mobile_fr3_duo.xml

和：

    mfr3duo_mujoco/models/mobile_fr3_duo.xml

两份模型。

---

## 2.5 与 `mujoco_simulation` 的边界

`mfr3duo_mujoco` 不重新实现：

    Simulation scheduler
    Physics thread
    CommandBuffer
    StateBuffer
    ComponentManager
    CameraRenderer
    Viewer
    Joint controller
    IMU component
    Camera component
    LiDAR component

而是组合：

    mujoco_simulation::Simulation

因此：

    mfr3duo_mujoco::Simulation
            │
            │ PImpl
            ▼
    mujoco_simulation::Simulation

这是：

    Facade
    +
    Semantic Adapter

关系，而不是两套 Simulation runtime。

---

## 2.6 `mujoco_simulation` 前置能力

为了完整支持 Mobile FR3 Duo，通用 `mujoco_simulation` 至少需要具备：

### Joint

支持：

    Active Joint
    Passive Joint

以及：

    Position
    Velocity
    Effort
    Hybrid
    None

支持：

    ShortestAngularDistance

支持：

    Active-only CommandBuffer
    Active + Passive StateBuffer

支持：

    transactional Joint command prepare/apply

这些要求由独立的：

    Joint 组件完善设计

负责。

---

### Base Body State

TMR chassis 使用：

    base_freejoint

但它不能使用现有：

    MobileBase

组件模拟。

TMR chassis 必须由：

    wheel actuator
        ↓
    contact
        ↓
    physics
        ↓
    base_freejoint

自然运动。

这条仿真与控制链完全由四个 Active Joint、Passive Joint、MuJoCo motor 和接触
动力学构成；`BodyState` 不参与 TMR command、Joint registration 或物理积分。
因此缺少 Body/FreeBody state component **不会阻塞基于 Joint 的 TMR 仿真**，只会
阻塞对外发布 `base_link` 的 ground-truth `BaseState`。

因此 `mujoco_simulation` 还需要提供一个通用的：

    Body
    RigidBody
    FreeBody

一类只读状态组件，用于读取：

    body position
    body orientation
    linear velocity
    angular velocity

推荐最终形成类似：

    BodyInfo
    BodyState

的通用 abstraction。

**当前状态：**现有 `mujoco_simulation` 尚未提供 `BodyInfo` / `BodyState`，其
`SimulationConfig` 也不能注册 Body/FreeBody 组件。因此这是
`mfr3duo_mujoco` 交付 `BaseState` 的硬前置条件，而不是本包可以临时绕过的
实现细节。该通用组件至少必须：

    以固定 body name 验证模型对象
    在世界坐标系发布 position / orientation
    发布世界坐标系 linear / angular velocity
    将 BodyState 纳入 RobotState 的不可变快照

`mfr3duo_mujoco` 不应该为了读取：

    base_link

而直接访问：

    mjModel
    mjData

也不应该错误地复用仅针对其他运动学模型设计的：

    MobileBase

组件。

因此：

> **通用 Body/FreeBase 状态读取能力是 `BaseState` 的前置扩展，而不是 TMR Joint
> 仿真的前置扩展。**

---

## 2.7 推荐目录结构

    mfr3duo_mujoco/
    ├── CMakeLists.txt
    ├── package.xml
    ├── README.md
    │
    ├── include/
    │   └── mfr3duo_mujoco/
    │       ├── simulation.hpp
    │       ├── simulation_config.hpp
    │       ├── simulation_status.hpp
    │       ├── export.hpp
    │       ├── version.hpp
    │       │
    │       └── data/
    │           ├── joint.hpp
    │           ├── tmr.hpp
    │           ├── spine.hpp
    │           ├── arm.hpp
    │           ├── gripper.hpp
    │           ├── imu.hpp
    │           ├── camera.hpp
    │           ├── simulation_state.hpp
    │           └── simulation_command.hpp
    │
    ├── src/
    │   ├── simulation.cpp
    │   ├── simulation_impl.hpp
    │   ├── simulation_impl.cpp
    │   │
    │   ├── model/
    │   │   ├── model_contract.hpp
    │   │   └── model_contract.cpp
    │   │
    │   ├── config/
    │   │   ├── runtime_config.hpp
    │   │   └── runtime_config.cpp
    │   │
    │   └── adapter/
    │       ├── command_adapter.hpp
    │       ├── command_adapter.cpp
    │       ├── state_adapter.hpp
    │       └── state_adapter.cpp
    │
    └── test/
        ├── unit/
        │   ├── runtime_config_test.cpp
        │   ├── command_adapter_test.cpp
        │   ├── state_adapter_test.cpp
        │   └── model_contract_test.cpp
        │
        └── integration/
            ├── simulation_lifecycle_test.cpp
            ├── simulation_reset_test.cpp
            ├── tmr_dynamics_test.cpp
            ├── dual_arm_test.cpp
            └── sensor_test.cpp

不建立：

    src/ros/
    src/node/
    launch/
    ros2_control/

这些属于其他 ROS 2 package。

---

# 3. 公共 API 与数据模型

## 3.1 API 设计原则

Public API 不允许直接暴露：

    mujoco_simulation::Simulation
    mujoco_simulation::JointInfo
    mujoco_simulation::JointState
    mujoco_simulation::RobotCommand
    mujoco_simulation::RobotState
    mjModel
    mjData
    ComponentId
    JointId

原因是：

> `mfr3duo_mujoco` 的 public API 应表达 Mobile FR3 Duo，而不是通用仿真框架内部结构。

因此 public headers 不包含：

    #include <mujoco/...>
    #include <mujoco_simulation/...>

`mujoco_simulation` 依赖完全隐藏在 PImpl 中。

---

## 3.2 基础数学类型

为了避免 public API 引入 Eigen 或 MuJoCo 类型，推荐使用：

    using Vector3d =
        std::array<double, 3>;

    using Quaterniond =
        std::array<double, 4>;

其中 Quaternion 顺序固定为：

    w, x, y, z

并在 public API 文档中明确。

---

## 3.3 JointMode

`mfr3duo_mujoco` 可以定义自己的机器人层控制模式：

    enum class JointMode
        : std::uint8_t {
        None = 0,
        Position,
        Velocity,
        Effort,
        Hybrid,
    };

它与：

    mujoco_simulation::JointMode

在：

    to_runtime_command
    from_runtime_state

内部进行转换。

这样避免底层 API 泄漏到 public header。

---

## 3.4 JointState

推荐：

    struct JointState {
        JointMode mode{
            JointMode::None};

        double position{0.0};

        double velocity{0.0};

        double effort{0.0};
    };

这里：

    position
    velocity
    effort

语义与底层 `mujoco_simulation::JointState` 保持一致。

其中：

    effort

表示：

> 当前 DoF 上 actuator generalized force contribution。

不是：

    contact force
    constraint force
    total generalized force

---

## 3.5 JointCommand

推荐：

    struct JointCommand {
        JointMode mode{
            JointMode::None};

        double position{0.0};

        double velocity{0.0};

        double effort{0.0};

        double stiffness{0.0};

        double damping{0.0};
    };

该结构主要用于：

    FR3
    Spine
    Gripper actuator

TMR command 则使用更强的语义类型，避免上层错误指定 TMR control mode。

---

## 3.6 TMR BaseState

以下 API 是可选的 ground-truth state 扩展；在 `mujoco_simulation` 完成
Body/FreeBody state component 之前，V1 实现不得将它标记为可用。这不影响仅基于
Joint 的 TMR 控制与物理仿真。

`BaseState` 只描述 TMR chassis 的 `base_link`，定义在 `data/tmr.hpp`；V1 不提供
独立的 `data/base.hpp`，也不提供通用 Base API。

推荐：

    struct BaseState {
        Vector3d position{};

        Quaterniond orientation{
            1.0, 0.0, 0.0, 0.0};

        Vector3d linear_velocity{};

        Vector3d angular_velocity{};
    };

该状态直接描述：

    base_link

在世界坐标系下的真实 MuJoCo 动力学状态。

它必须来源于：

    base_freejoint / body state

而不是 wheel odometry。

因此需要明确区分：

    BaseState
    → simulation ground truth

和：

    ROS odometry
    → controller / estimator result

二者不是同一个概念。

---

## 3.7 TmrState

推荐：

    struct TmrState {
        BaseState base;

        JointState front_steering;
        JointState front_drive;

        JointState rear_steering;
        JointState rear_drive;

        JointState rocker;

        JointState front_caster_steering;
        JointState front_caster_rolling;

        JointState rear_caster_steering;
        JointState rear_caster_rolling;
    };

`base` 仅在底层 Body/FreeBody state component 可用时填充真实 ground-truth。V1
尚不具备该能力，因此返回默认零位姿、单位四元数和零速度的占位值；不得以
odometry、Swerve FK 或估计值替代。调用方不得将该占位值当作真实 `base_link` 状态。

其中：

    front_steering
    front_drive
    rear_steering
    rear_drive

属于：

    Active Joint

而：

    rocker
    front_caster_steering
    front_caster_rolling
    rear_caster_steering
    rear_caster_rolling

属于：

    Passive Joint

但二者都属于：

    TmrState

---

## 3.8 TmrCommand

TMR 控制模式是固定的：

    steering → Position
    drive    → Velocity

因此不建议暴露：

    JointCommand front_steering

这种可以错误设置：

    Effort
    Hybrid
    Velocity

的接口。

推荐直接定义：

    struct TmrCommand {
        double front_steering_position{
            0.0};

        double front_drive_velocity{
            0.0};

        double rear_steering_position{
            0.0};

        double rear_drive_velocity{
            0.0};
    };

`to_runtime_command` 固定转换为：

    front steering
    → Position

    front drive
    → Velocity

    rear steering
    → Position

    rear drive
    → Velocity

这样：

> TMR 的控制模式约束由类型系统和 Adapter 固定，而不是交给调用者重复指定。

---

## 3.9 SpineState / SpineCommand

推荐：

    struct SpineState {
        JointState joint;
    };

    struct SpineCommand {
        JointCommand joint;
    };

Spine 当前为单一：

    Prismatic Active Joint

允许的控制模式由 model contract 决定。

---

## 3.10 ArmState / ArmCommand

定义：

    inline constexpr std::size_t
        kArmJointCount = 7;

    using ArmJointStates =
        std::array<
            JointState,
            kArmJointCount>;

    using ArmJointCommands =
        std::array<
            JointCommand,
            kArmJointCount>;

    struct ArmState {
        ArmJointStates joints;
    };

    struct ArmCommand {
        ArmJointCommands joints;
    };

数组顺序固定为：

    joint1
    joint2
    joint3
    joint4
    joint5
    joint6
    joint7

不在 public API 中使用：

    JointId

访问。

---

## 3.11 GripperState / GripperCommand

当前模型使用两指夹爪，并由一个 actuator 驱动主动 finger joint，通过 equality constraint 同步另一侧 finger。

推荐机器人层状态：

    struct GripperState {
        JointState left_finger;
        JointState right_finger;
    };

控制：

    struct GripperCommand {
        JointCommand actuator;
    };

其中：

    actuator

对应该 gripper 唯一主动 finger actuator。

上层：

    Franka Hand action semantics
    width
    speed
    force

不由 `mfr3duo_mujoco` 实现。

这些语义应该由：

    mfr3duo_hardware
    gripper controller

转换为底层 Joint command。

这样未来更换：

    Franka Hand
    Robotiq

时不会把 ROS Action 逻辑固化进仿真核心。

---

## 3.12 ImuState

推荐：

    struct ImuState {
        double timestamp{0.0};

        Quaterniond orientation{
            1.0, 0.0, 0.0, 0.0};

        Vector3d angular_velocity{};

        Vector3d linear_acceleration{};
    };

不包含：

    ROS Header
    frame_id
    covariance

这些属于 ROS sensor integration 层。

---

## 3.13 RobotState

实时主状态定义为：

    struct RobotState {
        double time{0.0};

        std::uint64_t step_count{0};

        TmrState tmr;

        SpineState spine;

        ArmState left_arm;
        ArmState right_arm;

        GripperState left_gripper;
        GripperState right_gripper;

        ImuState imu;
    };

这里刻意不包含：

        Camera image
        Depth image
    Contact list

原因是这些数据：

    数据量大
    更新频率不同
    生命周期不同

不适合进入 1 kHz realtime robot state snapshot。

---

## 3.14 RobotCommand

推荐：

    struct RobotCommand {
        TmrCommand tmr;

        SpineCommand spine;

        ArmCommand left_arm;
        ArmCommand right_arm;

        GripperCommand left_gripper;
        GripperCommand right_gripper;
    };

它表示：

> **Mobile FR3 Duo 当前全部 Active Joint 的完整命令快照。**

因此：

    RobotCommand

只包含：

    Active Joint

不包含：

    rocker
    caster

等 Passive Joint。

这与通用 `mujoco_simulation` 中：

    CommandDomain = Active Joint

的设计完全一致。

---

## 3.15 Camera 与未来 LiDAR

Camera 属于高带宽异步状态，不进入：

    RobotState

推荐通过独立接口读取。

可以定义：

    enum class CameraId {
        BaseFrontColor,
        BaseFrontDepth,

        BaseRearColor,
        BaseRearDepth,

        BaseLeftColor,
        BaseLeftDepth,

        BaseRightColor,
        BaseRightDepth,

        LeftWristColor,
        LeftWristDepth,

        RightWristColor,
        RightWristDepth,

        HeadLeft,
        HeadRight,
    };

Camera frame 使用自己的：

    CameraFrame

类型，不使用：

    sensor_msgs::msg::Image

V1 使用共享只读快照读取高带宽数据：

    std::shared_ptr<const CameraFrame>

调用者不得修改快照，也不得假定 runtime 会永久保留它；该约定避免在实时路径复制
RGB 和 depth 数据。

LiDAR 在 V1 **暂不支持**。当前 canonical MJCF 只有 LiDAR 安装 body/site，
没有 `mujoco_simulation::LidarInfo` 所需的完整 rangefinder 传感器集合。因此 V1
不定义 `LidarId`，也不注册 LiDAR component。未来启用前必须先在
`mfr3duo_description` 增加 canonical rangefinder 集合，再定义扫描几何、传感器
名称前缀、量程、角度和更新周期的模型契约。

---

## 3.16 SimulationStatus

推荐与底层生命周期语义保持一致：

    enum class SimulationStatus {
        Uninitialized,
        Stopped,
        Running,
        Paused,
        Stopping,
        Error,
    };

Adapter 内部负责转换：

    mujoco_simulation::SimulationStatus
        ↓
    mfr3duo_mujoco::SimulationStatus

---

## 3.17 SimulationConfig

Public config 不暴露：

    mujoco_simulation::SimulationConfig

推荐：

    struct SensorConfig {
        double imu_period{
            0.001};

        double camera_period{
            1.0 / 30.0};

        bool enable_cameras{
            true};
    };

    struct SimulationConfig {
        std::string model_path;

        std::string initial_keyframe{
            "home"};

        double physics_period{
            0.001};

        double viewer_period{
            1.0 / 60.0};

        double joint_period{
            0.001};

        double base_state_period{
            0.001};

        SensorConfig sensors;
    };

其中：

    model_path

必须是完整路径。

不支持：

    package://
    ament package name
    ROS substitution

这些由上层处理。

---

## 3.18 Simulation Public API

最终推荐：

    namespace mfr3duo_mujoco {

    class Simulation {
     public:
        Simulation();
        ~Simulation();

        Simulation(
            const Simulation&) = delete;

        Simulation&
        operator=(
            const Simulation&) = delete;

        Simulation(
            Simulation&&) = delete;

        Simulation&
        operator=(
            Simulation&&) = delete;

        bool initialize(
            const SimulationConfig& config);

        bool shutdown();

        bool start();

        bool stop();

        bool pause();

        bool resume();

        bool reset();

        bool reset(
            const std::string& keyframe);

        bool step(
            std::size_t count = 1);

        bool write_command(
            const RobotCommand& command);

        bool read_state(
            RobotState& state) const;

        std::shared_ptr<const CameraFrame>
        read_camera(CameraId id) const;

        bool read_contacts(
            ContactStates& contacts) const;

        SimulationStatus
        status() const;

        double time() const;

        std::uint64_t
        step_count() const;

     private:
        class Impl;

        std::unique_ptr<Impl> impl_;
    };

    }

V1 不提供：

    write_joint(id, ...)
    read_joint(id, ...)
    raw_model()
    raw_data()
    component_id()
    actuator_id()

避免 public API 回退到通用底层 abstraction。

---

# 4. 模型契约与配置构建

## 4.1 Model Contract

`mfr3duo_mujoco` 不负责创建模型。

它负责定义：

> **一个合法 Mobile FR3 Duo MJCF 必须提供哪些物理实体。**

模型契约集中定义在：

    src/model/model_contract.hpp

禁止将名字散落在：

    simulation.cpp
    state_adapter.cpp
    command_adapter.cpp
    runtime_config.cpp

各处。

---

## 4.2 Core TMR Contract

要求存在：

    body:
      base_link

    free joint:
      base_freejoint

Active TMR joints：

    tmrv0_2_joint_0
    tmrv0_2_joint_1
    tmrv0_2_joint_2
    tmrv0_2_joint_3

语义固定：

    tmrv0_2_joint_0
    → front steering
    → Active
    → Position
    → ShortestAngularDistance

    tmrv0_2_joint_1
    → front drive
    → Active
    → Velocity

    tmrv0_2_joint_2
    → rear steering
    → Active
    → Position
    → ShortestAngularDistance

    tmrv0_2_joint_3
    → rear drive
    → Active
    → Velocity

Passive joints：

    rocker_arm_joint

    caster_front_left_steering_joint
    caster_front_left_joint

    caster_rear_right_steering_joint
    caster_rear_right_joint

这些必须：

    Passive
    no actuator

---

## 4.3 TMR Actuator Contract

当前 canonical MJCF 已定义以下四个 direct-force MuJoCo motor，
`to_runtime_config` 必须验证并使用它们：

    tmrv0_2_joint_0_motor
    tmrv0_2_joint_1_motor
    tmrv0_2_joint_2_motor
    tmrv0_2_joint_3_motor

分别对应：

    joint_0
    joint_1
    joint_2
    joint_3

并使用：

    direct-force MuJoCo motor

而不是 MuJoCo 内建：

    position actuator
    velocity actuator

因为 Position / Velocity controller 已由：

    mujoco_simulation::Joint

实现。

---

## 4.4 Spine Contract

要求存在：

    franka_spine_vertical_joint

以及对应：

    franka_spine_motor

Spine：

    Active
    Prismatic

具体：

    allowed_modes
    default_mode
    gains
    gravity compensation

由 `to_runtime_config` 的 Mobile FR3 Duo control profile 定义。

---

## 4.5 Dual FR3 Contract

要求存在：

    left_fr3v2_1_joint1
    ...
    left_fr3v2_1_joint7

以及：

    right_fr3v2_1_joint1
    ...
    right_fr3v2_1_joint7

每个 Active Joint 必须对应唯一 actuator：

    left_fr3v2_1_joint1_motor
    ...
    left_fr3v2_1_joint7_motor

    right_fr3v2_1_joint1_motor
    ...
    right_fr3v2_1_joint7_motor

Arm array 顺序必须与：

    joint1 ... joint7

一致。

---

## 4.6 Gripper Contract

当前 Franka Hand 模型要求：

    left_fr3v2_1_finger_joint1
    left_fr3v2_1_finger_joint2

    right_fr3v2_1_finger_joint1
    right_fr3v2_1_finger_joint2

并通过 equality constraint 实现 finger coupling。

Active actuator：

    left_fr3v2_1_finger_motor

    right_fr3v2_1_finger_motor

Passive/coupled finger 不单独接受 command。

---

## 4.7 Sensor Contract

核心小型状态至少要求：

    Active / Passive Joint state
    IMU

`BaseState` 是可选 ground-truth 扩展；缺少底层 BodyState 时不得阻止基于 Joint
的 TMR 初始化或仿真。

Camera 根据：

    SensorConfig

决定是否注册。

如果：

    enable_cameras == true

则配置中要求的所有 Camera 必须存在。

禁止：

    configured but missing
    → silently ignored

正确行为：

    initialize() == false

并输出明确缺失对象名称。

当前 IMU contract 固定为：

    imu_orientation
    → framequat / orientation

    imu_angular_velocity
    → gyro / angular velocity

    imu_linear_acceleration
    → accelerometer / linear acceleration

当前 camera role 直接映射到 canonical MJCF camera 名称：

    Base{Front,Rear,Left,Right}{Color,Depth}
    → camera_{front,rear,left,right}_{color,depth}

    LeftWrist{Color,Depth}
    → d435_left_{rgb,depth}

    RightWrist{Color,Depth}
    → d435_right_{rgb,depth}

    Head{Left,Right}
    → head_zed_{left,right}

V1 不验证或注册 LiDAR。其 future contract 只能在 canonical MJCF 已提供
rangefinder 集合后建立。

---

## 4.8 Keyframe Contract

默认：

    initial_keyframe = "home"

因此 canonical MJCF 应至少提供：

    home

如果指定：

    transport
    manipulation
    wide_workspace
    ...

则该 keyframe 必须真实存在。

如果：

    initial_keyframe

不存在：

    initialize() == false

而不是 fallback 到：

    qpos = 0

避免机器人在非物理姿态启动。

---

## 4.9 Physical Parameters Ownership

必须明确不同配置的所有权。

### `mfr3duo_description`

拥有物理模型参数：

    mass
    inertia
    joint axis
    joint range
    collision geometry
    contact geometry
    wheel radius
    friction
    damping/frictionloss
    actuator ctrlrange
    actuator forcerange
    camera pose
    camera intrinsic approximation
    sensor pose
    equality constraints

`mfr3duo_mujoco` 不在 runtime 中覆盖这些参数。

---

### `mfr3duo_mujoco`

拥有运行和控制适配参数：

    component ID
    component role
    default Joint mode
    allowed Joint modes
    Position/Velocity controller gains
    gravity compensation policy
    component update period
    camera render period
    scheduler period

即：

    physical model
    → description

    runtime/control semantics
    → mfr3duo_mujoco

---

### 4.9.1 Franka 官方资料的使用边界

FR3 单臂的 joint limit、effort limit、运动学/动力学描述应以 Franka 官方
[`franka_description`](https://github.com/frankarobotics/franka_description) 和 FCI
产品文档为校核来源。现有 `mfr3duo_description` 的 canonical MJCF 仍是 runtime
唯一模型来源；发现与官方资料不一致时，应先修订并验证 canonical model，禁止由
`mfr3duo_mujoco` 在运行时覆盖物理参数。

Franka 官方
[`franka_ros2` controller configuration](https://github.com/frankarobotics/franka_ros2/blob/humble/franka_fr3_moveit_config/config/fr3_ros_controllers.yaml)
中的 FR3 controller gains 只能作为 arm 控制 profile 的初始调参参考。该配置是
ROS 2 effort trajectory controller 的参数，不能未经 MuJoCo 闭环验证直接视为
`mujoco_simulation::Joint` 的 Position / Velocity / Hybrid gains。

官方资料不定义以下项目，必须由本项目的模型、runtime contract 和测试确定：

    TMR steer/drive control profile 与轮地接触标定
    Spine / gripper control profile
    Mobile FR3 Duo component ID
    Camera snapshot API 与 frame mapping
    BodyState component API

---

## 4.10 Runtime Config 转换

`to_runtime_config` 的职责是：

    SimulationConfig
          +
    ModelContract
          ↓
    mujoco_simulation::SimulationConfig

内部：

    bool to_runtime_config(
        const SimulationConfig& input,
        mujoco_simulation::SimulationConfig& output);

它负责生成：

    ModelConfig
    SchedulerConfig
    ComponentConfigList

并注册：

    TMR Active Joint
    TMR Passive Joint
    Base Body（仅在底层 BodyState 能力可用时）
    Spine
    Left Arm
    Right Arm
    Left Gripper
    Right Gripper
    IMU
    Cameras

---

## 4.11 Internal Component IDs

底层 `mujoco_simulation` 按 component type 维护独立的 ID namespace：`JointId`、
`CameraId`、`ImuId` 等只须在同类型内唯一，不要求跨类型全局唯一。V1 不使用
`MobileBaseId` 或 `LidarId`。

所有 ID 必须：

    stable
    deterministic
    centralized

但：

> **只属于 `mfr3duo_mujoco` 内部实现，不进入 public API。**

建议集中定义：

    src/model/component_ids.hpp

### JointId

JointId 采用用户可读的 robot order：

| JointId | 语义 | MJCF joint | Actuation |
|---:|---|---|---|
| 0–6 | 左臂 joint1–joint7 | `left_fr3v2_1_joint1` – `left_fr3v2_1_joint7` | Active |
| 7–13 | 右臂 joint1–joint7 | `right_fr3v2_1_joint1` – `right_fr3v2_1_joint7` | Active |
| 14 | 升降 | `franka_spine_vertical_joint` | Active |
| 15 | 前转向 | `tmrv0_2_joint_0` | Active |
| 16 | 前驱动 | `tmrv0_2_joint_1` | Active |
| 17 | 后转向 | `tmrv0_2_joint_2` | Active |
| 18 | 后驱动 | `tmrv0_2_joint_3` | Active |
| 19 | rocker | `rocker_arm_joint` | Passive |
| 20 | 前 caster 转向 | `caster_front_left_steering_joint` | Passive |
| 21 | 前 caster 滚动 | `caster_front_left_joint` | Passive |
| 22 | 后 caster 转向 | `caster_rear_right_steering_joint` | Passive |
| 23 | 后 caster 滚动 | `caster_rear_right_joint` | Passive |
| 24–25 | 左夹爪 finger1–finger2 | `left_fr3v2_1_finger_joint1` – `left_fr3v2_1_finger_joint2` | Active, Passive |
| 26–27 | 右夹爪 finger1–finger2 | `right_fr3v2_1_finger_joint1` – `right_fr3v2_1_finger_joint2` | Active, Passive |

### CameraId 与 ImuId

CameraId 按 public `CameraId` enum 的声明顺序固定为 0–13：

| CameraId | role | MJCF camera | `frame_id` | `optical_frame_id` |
|---:|---|---|---|---|
| 0 | BaseFrontColor | `camera_front_color` | `camera_front_color_frame` | `camera_front_color_optical_frame` |
| 1 | BaseFrontDepth | `camera_front_depth` | `camera_front_depth_frame` | `camera_front_depth_optical_frame` |
| 2 | BaseRearColor | `camera_rear_color` | `camera_rear_color_frame` | `camera_rear_color_optical_frame` |
| 3 | BaseRearDepth | `camera_rear_depth` | `camera_rear_depth_frame` | `camera_rear_depth_optical_frame` |
| 4 | BaseLeftColor | `camera_left_color` | `camera_left_color_frame` | `camera_left_color_optical_frame` |
| 5 | BaseLeftDepth | `camera_left_depth` | `camera_left_depth_frame` | `camera_left_depth_optical_frame` |
| 6 | BaseRightColor | `camera_right_color` | `camera_right_color_frame` | `camera_right_color_optical_frame` |
| 7 | BaseRightDepth | `camera_right_depth` | `camera_right_depth_frame` | `camera_right_depth_optical_frame` |
| 8 | LeftWristColor | `d435_left_rgb` | `left_d435_link` | `left_d435_color_optical_frame` |
| 9 | LeftWristDepth | `d435_left_depth` | `left_d435_link` | `left_d435_depth_optical_frame` |
| 10 | RightWristColor | `d435_right_rgb` | `right_d435_link` | `right_d435_color_optical_frame` |
| 11 | RightWristDepth | `d435_right_depth` | `right_d435_link` | `right_d435_depth_optical_frame` |
| 12 | HeadLeft | `head_zed_left` | `head_zed_left_camera_frame` | `head_zed_left_camera_optical_frame` |
| 13 | HeadRight | `head_zed_right` | `head_zed_right_camera_frame` | `head_zed_right_camera_optical_frame` |

Color camera 固定 `enable_rgb=true`、`enable_depth=false`；Depth camera 固定
`enable_rgb=false`、`enable_depth=true`；Head camera 固定 RGB-only。每个 camera 的
width/height 必须与 canonical MJCF `resolution` 一致。

    ImuId 0
    → imu_orientation / imu_angular_velocity / imu_linear_acceleration

未来 Body component 增加后，在其独立 BodyId namespace 使用 `BodyId 0` 表示
`base_link`。V1 不预留跨类型的数字范围，也不允许重排以上已发布 ID。

`from_runtime_state` 和 `to_runtime_command` 只能使用：

    ModelContract / ComponentIds

不能直接出现 magic number。

---

## 4.12 模型验证原则

不建议 `mfr3duo_mujoco` 自己再次：

    mj_loadXML()

加载第二份模型用于验证。

推荐：

    to_runtime_config
        ↓
    完整生成所有 required component config
        ↓
    mujoco_simulation::Simulation::initialize()
        ↓
    各 Component 完成 name / actuator / type validation

即：

> **通过完整的 Component Contract 注册，让通用 runtime 在一次模型加载过程中完成结构验证。**

`mfr3duo_mujoco` 只负责：

    semantic contract validation

`mujoco_simulation` 负责：

    physical model lookup / validation

这样避免：

    duplicate model loading
    duplicate MJCF parser
    duplicated MuJoCo model ownership

---

# 5. 状态、命令与运行时设计

## 5.1 RobotState 转换

底层：

    mujoco_simulation::RobotState

以：

    Component ID

组织状态。

但上层需要：

    state.tmr.front_steering
    state.left_arm.joints[0]
    state.spine.joint

因此需要：

    from_runtime_state

职责：

    generic component state
            ↓
    Mobile FR3 Duo semantic state

推荐：

    bool from_runtime_state(
        const mujoco_simulation::RobotState& input,
        RobotState& output);

---

## 5.2 RobotState 转换不执行动力学计算

`from_runtime_state` 只能：

    locate
    validate
    copy
    convert type

禁止：

    calculate odometry
    integrate velocity
    calculate Swerve FK
    modify state
    estimate pose

若底层 BodyState 能力可用：

    TmrState::base

必须直接来源于：

    base_link ground-truth BodyState

而不是根据：

    steering/wheel states

重新计算。

若该能力尚不可用，`from_runtime_state` 仍必须完整转换 TMR 的 Active/Passive Joint
状态；它不得以 wheel odometry 或 Swerve FK 伪造 `BaseState`。

---

## 5.3 RobotCommand 转换

Public：

    RobotCommand

需要转换为：

    mujoco_simulation::RobotCommand

因此：

    bool to_runtime_command(
        const RobotCommand& input,
        mujoco_simulation::RobotCommand& output);

---

## 5.4 TMR Command Mapping

固定映射：

    TmrCommand.front_steering_position
        ↓
    tmrv0_2_joint_0
    Position

    TmrCommand.front_drive_velocity
        ↓
    tmrv0_2_joint_1
    Velocity

    TmrCommand.rear_steering_position
        ↓
    tmrv0_2_joint_2
    Position

    TmrCommand.rear_drive_velocity
        ↓
    tmrv0_2_joint_3
    Velocity

因此：

    mfr3duo_mujoco

不执行：

    vx/vy/wz → wheel

转换。

---

## 5.5 Passive Joint 不进入 RobotCommand 转换

`to_runtime_command` 永远不会生成：

    rocker command
    caster command

即：

    RobotCommand
    =
    Active joints only

与通用 runtime：

    CommandDomain = Active

保持一致。

---

## 5.6 Whole-Frame Command

Public API 推荐只提供：

    write_command(
        const RobotCommand&)

而不提供：

    write_front_drive()
    write_left_joint3()
    write_spine()
    write_joint(id)

原因是：

> Mobile FR3 Duo 最终需要 whole-body coordinated control。

使用完整 command frame 能够：

    保持 subsystem command 一致性
    支持底层 transactional commit
    避免一个 control cycle 内出现 partial whole-body command

底层：

    RobotCommand
        ↓
    prepare all Joint commands
        ↓
    apply all

完成事务提交。

---

## 5.7 Command Persistence

`mujoco_simulation::CommandBuffer` 负责 persistent command。

`mfr3duo_mujoco` 不维护第二套：

    command cache
    command mutex
    last command buffer

避免形成：

    mfr3duo command buffer
            ↓
    mujoco_simulation command buffer

双层状态。

因此：

> `to_runtime_command` 只负责转换，不负责持久化。

---

## 5.8 State Snapshot

同样：

    mujoco_simulation::StateBuffer

是唯一底层状态 snapshot source。

`mfr3duo_mujoco` 不建立第二套 physics state ownership。

它可以：

    read generic snapshot
        ↓
    convert semantic snapshot

如果后续性能分析表明完整转换开销明显，可增加：

    shared semantic snapshot

缓存。

但不得在 V1 提前建立复杂的第二套 buffer。

---

## 5.9 高带宽传感器

Camera 不进入：

    RobotState

避免每次：

    read_state()

复制：

    RGB image
    Depth image

正确接口：

    read_camera()

`read_camera()` 返回：

    std::shared_ptr<const CameraFrame>

即共享不可变快照，避免大数据重复复制。LiDAR 的同类 API 留待其模型前置条件
满足后再加入，不属于 V1。

---

## 5.10 生命周期

`mfr3duo_mujoco::Simulation` 生命周期直接包装底层：

    Uninitialized
         │
         │ initialize
         ▼
       Stopped
         │
         │ start
         ▼
       Running
         │
      ┌──┴───┐
      │      │
    pause   stop
      │      │
      ▼      ▼
    Paused  Stopped
      │
    resume
      │
      ▼
    Running

任何不可恢复 runtime failure：

    → Error

---

## 5.11 Initialize

推荐流程：

    Simulation::initialize(config)
        │
        ├── validate public config
        │
        ├── to_runtime_config()
        │
        ├── create underlying Simulation
        │
        ├── underlying.initialize()
        │
        ├── verify required model contract
        │
        ├── configure adapters
        │
        ├── initial state read
        │
        └── success → Stopped

初始化必须具有事务语义：

> 失败后对象保持可重新 `initialize()` 的干净状态。

不能留下：

    partially initialized runtime
    partially registered component
    stale state
    stale command

---

## 5.12 Reset

`reset()` 负责恢复：

    initial_keyframe

`reset(keyframe)` 负责恢复指定 keyframe。

流程：

    pause/stop-safe reset
        ↓
    mujoco_simulation reset
        ↓
    reset command buffer
        ↓
    Active Joint reset command
        ↓
    Passive Joint state naturally reset
        ↓
    StateAdapter refresh

Passive：

    rocker
    caster

不得生成任何 command。

---

## 5.13 Step Mode

保留：

    step(count)

用于：

    unit test
    integration test
    deterministic test
    controller debugging

在：

    Running

状态下是否允许 `step()`，保持与底层 `mujoco_simulation` 一致。

`mfr3duo_mujoco` 不自行改变 generic Simulation lifecycle 规则。

---

## 5.14 Threading

`mfr3duo_mujoco` 不创建自己的：

    physics thread
    sensor thread
    camera thread
    viewer thread

这些线程全部由：

    mujoco_simulation

管理。

因此：

    mfr3duo_mujoco::Simulation::Impl

主要包含：

    std::unique_ptr<
        mujoco_simulation::Simulation>
        simulation_;

    // to_runtime_config() is a stateless function.

    // Stateless conversion functions are called directly.

而不是：

    std::thread
    condition_variable
    scheduler

---

## 5.15 实时控制路径

典型 1 kHz physics loop：

    MuJoCo state
        ↓
    mujoco_simulation::StateBuffer
        ↓
    mfr3duo_mujoco::read_state()
        ↓
    mfr3duo_hardware::read()
        ↓
    controller_manager::update()
        ↓
    mfr3duo_hardware::write()
        ↓
    mfr3duo_mujoco::write_command()
        ↓
    mujoco_simulation::CommandBuffer
        ↓
    Joint prepare/apply
        ↓
    mjData.ctrl
        ↓
    mj_step()

如果 physics：

    1 kHz

而 WBC：

    500 Hz

则：

    每两个 physics step
    更新一次 WBC command

中间 step：

    保持上一完整 command snapshot

不需要额外 ROS Topic。

---

## 5.16 无 ROS Topic 实时闭环

禁止实现：

    mfr3duo_mujoco
        ↓ ROS Topic
    mfr3duo_hardware

实时闭环必须使用：

    native C++ call

数据路径：

    controller
        ↓
    ros2_control interface
        ↓
    mfr3duo_hardware
        ↓
    native C++
        ↓
    mfr3duo_mujoco
        ↓
    native C++
        ↓
    mujoco_simulation

ROS Topic 只用于：

    telemetry
    debugging
    visualization
    sensors
    high-level command

而不是 1 kHz Joint control transport。

---

## 5.17 错误处理

继续采用：

    bool + log

风格。

Public control path 不使用异常作为正常错误机制。

例如：

    bool initialize(...)
    bool write_command(...)
    bool read_state(...)
    bool reset(...)

失败：

    return false

并记录：

    subsystem
    component
    expected name
    actual problem

例如：

    missing required joint:
    tmrv0_2_joint_0

而不是只输出：

    initialization failed

---

# 6. 与系统其他模块的集成

## 6.1 与 `mfr3duo_description`

`mfr3duo_description` 是：

> **唯一 canonical robot model source。**

`mfr3duo_mujoco` 只定义：

    expected names
    expected roles
    control mapping

不保存：

    mesh
    XML
    physical inertial parameters

二者关系：

    mfr3duo_description
          │
          │ MJCF
          ▼
    mfr3duo_mujoco
          │
          │ semantic contract
          ▼
    mujoco_simulation

---

## 6.2 Description Compatibility

`mfr3duo_mujoco` 应在 `package.xml` 与发布说明中明确声明所支持的
`mfr3duo_description` package 版本范围；V1 的契约基线为当前 canonical model
对应的 `mfr3duo_description` `0.1.x`。不得仅以仓库分支或运行时名称猜测表示兼容。

不要通过运行时：

    猜测 Joint 名称
    fallback alias
    模糊匹配

兼容未知模型。

如果 model contract 发生 breaking change：

    Joint rename
    actuator rename
    sensor rename
    device replacement

必须同步提升兼容边界、更新模型契约测试，并在两包的发布说明中记录 breaking
change。

---

## 6.3 与 `mujoco_simulation`

关系：

    mfr3duo_mujoco
    → robot-specific semantic layer

    mujoco_simulation
    → robot-independent simulation runtime

任何可以被其他机器人复用的能力，例如：

    Passive Joint
    BodyState
    generic contact
    command transaction
    sensor component
    scheduler

必须优先实现到：

    mujoco_simulation

而不是：

    mfr3duo_mujoco

---

## 6.4 与 `mfr3duo_hardware`

`mfr3duo_hardware` 只需要理解：

    RobotState
    RobotCommand

不需要知道：

    MuJoCo JointId
    ComponentId
    actuator name
    camera name
    XML hierarchy

典型结构：

    class Mfr3DuoSystem
      : public hardware_interface::SystemInterface
    {
        mfr3duo_mujoco::Simulation
            simulation_;

        mfr3duo_mujoco::RobotState
            state_;

        mfr3duo_mujoco::RobotCommand
            command_;
    };

ROS 2 resource mapping：

    ros2_control
        ↓
    RobotCommand
        ↓
    mfr3duo_mujoco

---

## 6.5 TMR 与 Franka Swerve Controller

TMR 控制链保持：

    /cmd_vel
        ↓
    SwerveDriveController
        ↓
    SwerveIKController
        ↓
    joint_0 position
    joint_1 velocity
    joint_2 position
    joint_3 velocity
        ↓
    mfr3duo_hardware
        ↓
    TmrCommand
        ↓
    mfr3duo_mujoco
        ↓
    Joint
        ↓
    MuJoCo physics

因此：

    Swerve IK

不进入：

    mfr3duo_mujoco

---

## 6.6 WBC 模式

未来 WBC 输出：

    base vx
    base vy
    base wz
    spine velocity
    left arm velocity
    right arm velocity

其中 TMR：

    vx/vy/wz

仍然通过：

    Swerve IK

转换为：

    steering position
    wheel velocity

然后进入：

    TmrCommand

因此 `mfr3duo_mujoco` 不需要理解：

    18-DoF reduced WBC vector

它只理解：

> **物理执行层的 Joint command。**

这是 WBC model 和 physics plant model 解耦的重要边界。

---

## 6.7 MoveIt

MoveIt 输出：

    RobotTrajectory

不直接发送给：

    mfr3duo_mujoco

路径应为：

    MoveIt
        ↓
    controller
        ↓
    ros2_control
        ↓
    mfr3duo_hardware
        ↓
    mfr3duo_mujoco

因此：

    trajectory interpolation
    FollowJointTrajectory
    MoveIt controller manager

不属于本包。

---

## 6.8 Nav2

Nav2：

    cmd_vel
      ↓
    Swerve Controller
      ↓
    mfr3duo_hardware
      ↓
    TmrCommand

同样：

    Nav2
    cmd_vel subscription

不进入：

    mfr3duo_mujoco

---

## 6.9 Sensor ROS Bridge

`mfr3duo_mujoco` 在 V1 只提供：

    ImuState
    CameraFrame

ROS 层负责转换为：

    sensor_msgs/Imu
    sensor_msgs/Image
    sensor_msgs/CameraInfo

该转换可以位于：

    mfr3duo_hardware

或未来独立：

    mfr3duo_sensor_bridge

但不能进入 `mfr3duo_mujoco` core。

未来 LiDAR 支持启用后，才由该 ROS bridge 将其独立状态转换为
`sensor_msgs/LaserScan`。

---

# 7. 测试、开发计划与验收标准

## 7.1 Unit Test

### ModelContract

验证：

    Joint names
    actuator names
    component roles
    camera names
    IMU sensor names and sensor types
    component IDs

不存在：

    duplicate semantic ID
    duplicate component ID
    missing required role

---

### Runtime Config 转换

输入：

    SimulationConfig

输出必须包含：

    4 TMR Active Joint
    TMR Passive Joint
    Spine
    14 FR3 Joint
    Gripper Joint
    Base Body（仅在底层 BodyState 能力可用时）
    IMU
    configured Cameras

并验证：

    TMR steering
    → Position
    → ShortestAngularDistance

    TMR drive
    → Velocity

    rocker/caster
    → Passive

---

### CommandAdapter

验证：

    TmrCommand
        ↓
    exactly four JointCommand

且：

    steering mode == Position
    drive mode == Velocity

验证：

    rocker/caster

永远不会产生 command。

---

### Arm Mapping

验证：

    left_arm.joints[0]
    → left_joint1

...

    left_arm.joints[6]
    → left_joint7

右臂同理。

---

### StateAdapter

构造底层 component state：

    Active
    Passive
    Arms
    Spine
    Base（仅在底层 BodyState 能力可用时）

验证正确写入：

    RobotState

并确保：

    state array ordering

稳定。

在底层 BodyState 不可用的构建中，额外验证 `state.tmr.base` 为默认占位值，
同时其余 TMR Joint 状态仍可正常读取。

---

## 7.2 Lifecycle Integration Test

覆盖：

    initialize
    start
    pause
    resume
    stop
    reset
    reset(keyframe)
    shutdown

验证非法状态调用正确失败。

例如：

    start before initialize
    → false

    initialize twice
    → false

具体语义与底层 `mujoco_simulation` 保持一致。

---

## 7.3 Model Contract Failure Test

删除：

    tmrv0_2_joint_0

要求：

    initialize == false

删除：

    tmrv0_2_joint_0_motor

要求：

    initialize == false

给 rocker 添加 actuator：

    initialize == false

将 steering 设为 limited：

    initialize == false

删除：

    base_freejoint

要求：

    initialize == false

---

## 7.4 TMR Dynamics Test

必须测试：

    forward
    backward
    lateral
    rotate
    combined vx/vy/wz
    stop
    reverse

验证：

    steering angle
    wheel velocity
    base pose
    base velocity

符合预期。

---

## 7.5 Continuous Steering Test

测试：

    +π → -π
    -π → +π
    multi-turn state

确保：

    steering

不发生无意义整圈旋转。

---

## 7.6 Passive Dynamics Test

验证：

    rocker
    caster steering
    caster rolling

能够因：

    contact
    gravity
    chassis movement

自然运动。

禁止任何：

    command

作用到这些 Joint。

---

## 7.7 Base Dynamics Test

这是 TMR 仿真的关键验收测试。

测试过程中禁止：

    qpos[base_freejoint] direct write
    qvel[base_freejoint] direct write

只发送：

    steering position
    drive velocity

然后确认：

    base_link

真实发生：

    translation
    rotation

证明运动来自：

    actuator
    +
    contact
    +
    dynamics

而不是：

    KinematicBaseController

---

## 7.8 Dual Arm Test

验证：

    left arm
    right arm

可以同时：

    Position
    Velocity
    Effort
    Hybrid

控制。

验证：

    gravity compensation

符合预期。

验证 command frame：

    left + right

同周期提交。

---

## 7.9 Spine Test

验证：

    position hold
    velocity control
    gravity/load behavior
    range limit

保证 Spine 在机器人上部负载下保持稳定。

---

## 7.10 Gripper Test

验证：

    finger coupling
    actuator command
    finger states

确保：

    only active finger actuator

接收 command。

---

## 7.11 Sensor Test

IMU：

    orientation
    angular velocity
    linear acceleration

Camera：

    correct camera role
    resolution
    frame update period

并验证：

    Camera high-bandwidth data

不会进入实时：

    RobotState

且 `read_camera()` 返回 shared immutable snapshot，不走大对象复制路径。

LiDAR 测试不属于 V1；在模型加入完整 rangefinder contract 后，与 LiDAR 功能
一起新增。

---

## 7.12 Reset / Keyframe Test

至少验证：

    home
    transport
    manipulation
    wide_workspace

若这些 keyframe 属于当前 canonical model。

Reset 后：

    Active Joint command
    StateBuffer
    Passive Joint
    BaseState

必须与目标 keyframe 一致。

---

## 7.13 Deterministic Step Test

使用：

    Simulation::step()

执行相同：

    initial state
    command sequence

应得到可接受误差范围内一致的：

    joint state
    base state
    simulation time

用于 CI 和 regression test。

---

## 7.14 性能测试

目标 physics：

    1 kHz

重点验证：

    mfr3duo_mujoco

不会增加：

    physics thread
    additional mutex
    per-cycle XML lookup
    per-cycle name lookup
    per-cycle heap growth

CommandAdapter / StateAdapter 应主要执行：

    fixed-size array copy
    indexed lookup
    enum conversion

不进行：

    string lookup
    mj_name2id
    map construction

---

## 7.15 开发阶段

### Phase 1：前置能力完成

首先完成 `mujoco_simulation`：

    Active / Passive Joint
    active-only command mapping
    transactional command
    continuous steering position error

TMR 仿真必须首先仅依赖：

    Active / Passive Joint
    Joint command mapping
    MuJoCo motor
    wheel-ground contact
    base_freejoint physics

`BodyState / FreeBaseState` 可以并行或后续补充；它需要完成固定 body name 验证、
世界坐标 pose/velocity 读取和 `RobotState` 快照集成。未完成前，
`mfr3duo_mujoco` 不能声称已提供 BaseState，但可以交付基于 Joint 的 TMR 仿真。

阶段验收：

> 通用 `mujoco_simulation` 已经能够表达完整 TMR 物理 plant，而不知道 TMR 是什么。

---

### Phase 2：Package Skeleton 与 Public API

创建：

    mfr3duo_mujoco

完成：

    CMakeLists
    package.xml

完成 public：

    Simulation
    SimulationConfig
    SimulationStatus
    RobotState
    RobotCommand

以及 subsystem data types。

阶段验收：

> Public header 不包含 ROS、MuJoCo 或 `mujoco_simulation` 类型。

---

### Phase 3：ModelContract 与 Runtime Config 转换

建立：

    ModelContract

集中定义：

    Joint
    actuator
    body
    sensor
    keyframe
    component ID

完成：

    to_runtime_config

构造完整：

    mujoco_simulation::SimulationConfig

阶段验收：

> 一个 `SimulationConfig` 即可完整初始化 Mobile FR3 Duo，不需要调用者手工注册 Component。

---

### Phase 4：State / Command Adapter

完成：

    CommandAdapter
    StateAdapter

实现：

    TMR
    Spine
    Dual FR3
    Gripper
    Base（在底层 BodyState 可用后）
    IMU

映射。

阶段验收：

> Public API 中不出现任何 raw ComponentId / JointId。

---

### Phase 5：Camera Integration

完成：

    Camera

语义映射和读取 API。

高带宽传感器保持独立读取。

LiDAR 不属于 V1；在 canonical MJCF 提供完整 rangefinder 集合之前，不创建
LiDAR mapping、读取 API 或验收测试。

---

### Phase 6：Simulation Facade

完成：

    initialize
    shutdown
    start
    stop
    pause
    resume
    reset
    step
    read_state
    write_command
    read_camera

内部完整组合：

    mujoco_simulation::Simulation

阶段验收：

> 可以在纯 C++ standalone test 中运行完整 Mobile FR3 Duo MuJoCo 仿真。

---

### Phase 7：TMR 动力学验证

当前 canonical MJCF 已包含：

    4 TMR direct-force motor actuators

验证 `to_runtime_config` 对这四个 actuator 的映射，并删除生产控制路径对：

    KinematicBaseController

的依赖。随后验证：

    physical steering
    physical wheel rotation
    wheel-ground contact
    chassis dynamics

验证：

    forward
    lateral
    rotate

---

### Phase 8：`mfr3duo_hardware` 集成

实现：

    mfr3duo_hardware
        ↓
    mfr3duo_mujoco::Simulation

映射：

    ros2_control state interfaces
        ←
    RobotState

    ros2_control command interfaces
        →
    RobotCommand

然后接入：

    Franka controllers
    SwerveDriveController
    SwerveIKController

---

## 7.16 最终验收标准

`mfr3duo_mujoco` 只有同时满足以下条件才视为完成：

1. `mfr3duo_mujoco` 是独立 ROS 2 ament package；
2. 核心 library 不依赖 `rclcpp`；
3. 核心 library 不依赖 `hardware_interface`；
4. 核心 library 不创建 ROS Node；
5. 核心 library 不发布 Topic；
6. 模型只来自 `mfr3duo_description`；
7. `mfr3duo_mujoco` 不保存 MJCF 副本；
8. 模型路径由调用者显式传入；
9. public headers 不暴露 MuJoCo 类型；
10. public headers 不暴露 `mujoco_simulation` 类型；
11. `Simulation` 使用 PImpl；
12. `mujoco_simulation::Simulation` 是唯一底层 runtime；
13. `mfr3duo_mujoco` 不创建额外 physics thread；
14. `mfr3duo_mujoco` 不建立第二套 CommandBuffer；
15. `mfr3duo_mujoco` 不建立第二套 physics StateBuffer；
16. 所有模型名称集中在 ModelContract；
17. 不允许 model-name magic string 散落；
18. TMR 四个 steer/drive joint 为 Active Joint；
19. Rocker/Caster 为 Passive Joint；
20. Passive Joint 不进入 RobotCommand；
21. TMR Steering 固定映射 Position；
22. TMR Drive 固定映射 Velocity；
23. TMR Steering 使用 ShortestAngularDistance；
24. TMR Swerve IK 不进入本包；
25. `base_freejoint` 不直接接收 command；
26. TMR chassis 不通过直接写 qpos/qvel 运动；
27. TMR chassis 运动来自 actuator + contact + dynamics；
28. `KinematicBaseController` 不用于 production dynamic backend；
29. 当底层 BodyState 能力可用时，BaseState 来源于真实 body/freejoint state；
30. BaseState 不通过 wheel odometry 重建；不可用时 `TmrState::base` 为默认占位值；
31. FR3 双臂状态以固定 7 元数组提供；
32. public API 不要求调用者理解 Component ID；
33. `RobotCommand` 表示完整 Active Joint command snapshot；
34. command 写入最终使用底层 transactional Joint commit；
35. Camera 不进入 1 kHz `RobotState`；
36. Camera 使用共享只读快照的独立读取接口；
37. IMU 可以进入 realtime small-state snapshot；
38. 初始化严格验证必需模型契约；
39. 缺少 required Joint/Actuator/Sensor 时初始化失败；
40. 初始化失败后对象保持干净可重新初始化；
41. 支持 `home` keyframe reset；
42. 支持显式 keyframe reset；
43. 支持 deterministic `step()`；
44. 可以在不启动 ROS 2 的情况下运行 unit/integration test；
45. 可以被 `mfr3duo_hardware` 通过 native C++ API 直接调用；
46. 1 kHz physics loop 不因本层增加额外锁或不可控动态分配；
47. FR3、Spine、TMR 和 Gripper 的 command/state 语义明确；
48. 模块职责与 `mfr3duo_description`、`mujoco_simulation`、`mfr3duo_hardware` 不重叠。
49. V1 不注册或暴露 LiDAR；只有 canonical MJCF 提供完整 rangefinder contract 后，
    才能增加 LiDAR component、API 与测试；
50. BaseState 依赖 `mujoco_simulation` 已提供的 Body/FreeBody state component；
    在此前置能力完成前，不将 BaseState 视为已交付能力。
51. TMR 仿真仅通过 Active/Passive Joint、MuJoCo motor、轮地接触和
    `base_freejoint` physics 实现；它不依赖 BodyState 或 MobileBase component。

---

## 7.17 开发前待定事项

以下事项尚不能从 Franka 官方单臂仿真资料直接获得。实现前必须在本项目中作出
明确决定，并把结论固化为 `ModelContract`、`to_runtime_config` 常量和对应测试；不得
由实现者临时猜测。

### P0：阻塞完整 runtime config / conversion 实现

1. **Joint control profile**：为 TMR、Spine、双 FR3 与 gripper 分别确定
   `allowed_modes`、`default_mode`、Position/Velocity/Hybrid gains、effort / velocity
   limits 和 gravity compensation。FR3 官方 limit 可作校核；所有 gain 必须经
   MuJoCo 闭环测试定版。
2. **Camera snapshot transport**：Camera role、ID、frame、pixel policy 与分辨率
   已由 4.11 固定；仍须在 `mujoco_simulation` 明确读取单个共享 `CameraState` 的
   public API，或定义等价的零大对象复制转换方式，再实现
   `std::shared_ptr<const CameraFrame>` facade。

### P1：不阻塞 Joint TMR 仿真，但阻塞完整状态与集成

4. **BodyState API**：在 `mujoco_simulation` 定义 Body component config、ID、
   `RobotState` 快照形式、世界坐标速度约定与 reset 语义。完成前
   `TmrState::base` 保持默认占位值。
5. **ros2_control resource mapping**：为 TMR、Spine、双臂和 gripper 固定 state /
   command interface 名称、接口类型与命名空间，确保 `mfr3duo_hardware` 无需接触
   MuJoCo ID。
6. **验收阈值**：为 TMR 的直行、横移、转向、组合速度、停止和连续转向定义可量化
   的位置/速度误差、稳定时间、允许滑移和 deterministic-step 容差。

### P2：后续能力

7. **LiDAR**：先在 canonical MJCF 加入完整 rangefinder 集合，再定义扫描布局、
   prefix、量程、角度、频率和 ROS bridge 行为。

待定事项关闭条件：对应 API/常量进入代码、模型契约测试覆盖缺失或错误配置，且
integration test 满足已确定的验收阈值。

---

# 最终设计结论

`mfr3duo_mujoco` 应正式定义为：

> **Mobile FR3 Duo 的 MuJoCo 仿真语义适配层。它基于通用 `mujoco_simulation` runtime，将 `mfr3duo_description` 提供的 Mobile FR3 Duo MJCF 模型映射为稳定的机器人级 State、Command、Lifecycle 和 Sensor C++ API，但不承担 ROS 2 集成、运动控制算法、Swerve IK 或 MuJoCo 通用运行时职责。**

最终软件边界冻结为：

    mfr3duo_description
            │
            │ canonical MJCF
            ▼
    ┌───────────────────────────┐
    │     mfr3duo_mujoco        │
    │                           │
    │ ModelContract             │
    │ to_runtime_config         │
    │ StateAdapter              │
    │ CommandAdapter            │
    │ Simulation Facade         │
    └─────────────┬─────────────┘
                  │
                  │ native C++
                  ▼
    ┌───────────────────────────┐
    │    mujoco_simulation      │
    │                           │
    │ Simulation                │
    │ Scheduler                 │
    │ ComponentManager          │
    │ CommandBuffer             │
    │ StateBuffer               │
    │ Joint / Body / Sensors    │
    │ CameraRenderer / Viewer   │
    └─────────────┬─────────────┘
                  │
                  ▼
                MuJoCo

上层：

    MoveIt / Nav2 / WBC
            │
            ▼
    ROS 2 Controllers
            │
            ▼
    mfr3duo_hardware
            │
            │ RobotState
            │ RobotCommand
            ▼
    mfr3duo_mujoco

TMR：

    vx / vy / wz
          ↓
    Swerve IK
          ↓
    steering position
    +
    drive velocity
          ↓
    TmrCommand
          ↓
    Joint
          ↓
    MuJoCo motor
          ↓
    wheel-ground contact
          ↓
    base_freejoint dynamics

双臂：

    controller command
          ↓
    ArmCommand
          ↓
    14 Active Joint
          ↓
    MuJoCo motor
          ↓
    FR3 dynamics

状态：

    MuJoCo physical state
          ↓
    mujoco_simulation
          ↓
    StateAdapter
          ↓
    RobotState
          ↓
    mfr3duo_hardware
          ↓
    ros2_control

这种结构实现了四个关键目标：

1. **模型唯一**：模型只属于 `mfr3duo_description`；
2. **仿真通用**：所有可复用 MuJoCo 能力只属于 `mujoco_simulation`；
3. **机器人语义集中**：Mobile FR3 Duo 的名称、设备、状态和命令映射只属于 `mfr3duo_mujoco`；
4. **ROS 2 解耦**：`mfr3duo_hardware` 负责 ROS 2，而 `mfr3duo_mujoco` 核心保持纯 C++。

因此，`mfr3duo_mujoco` 不应发展成第二套 MuJoCo 仿真框架，而应该始终保持：

> **薄而严格的 Mobile FR3 Duo semantic layer。**

后续任何新功能都应首先判断：

    是否是机器人无关能力？
        │
        ├── 是
        │    → mujoco_simulation
        │
        └── 否
             │
             ├── 是否是 Mobile FR3 Duo
             │   MuJoCo 模型语义？
             │       │
             │       ├── 是
             │       │    → mfr3duo_mujoco
             │       │
             │       └── 否
             │
             └── 是否属于 ROS 2 integration？
                     │
                     └── 是
                          → mfr3duo_hardware /
                            control / bringup

以此作为后续 `mfr3duo_mujoco` 开发和代码审查的长期架构约束。
