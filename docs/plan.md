# MFR3Duo ROS2 四层控制与任务接口设计方案

## 1. 总体架构、实施边界与运行约束

`mfr3duo_ros2` 的运行时能力划分为四层：

```text
mfr3duo_control
    “让这个设备执行某个基础命令”

mfr3duo_moveit
    “让这些 RobotGroup 联合运动到这些目标”

mfr3duo_nav
    “让机器人导航到这里”

mfr3duo_robot
    “让机器人完成这个任务”
```

编译与链接依赖按能力分别建立：

```text
mfr3duo_robot
    ├── mfr3duo_moveit → MoveIt / ROS 2 interfaces
    ├── mfr3duo_nav → Nav2 / ROS 2 interfaces
    └── mfr3duo_control → ros2_control / ROS 2 interfaces

mfr3duo_hardware
    ├── libmfr3duo_ros2_adapter → ROS 2 interfaces + libmfr3duo_hardware
    └── libmfr3duo_hardware → mfr3duo_mujoco（私有依赖）

mfr3duo_msgs
    → 共享的 ROS observation 消息与服务定义
```

`mfr3duo_msgs` 保存参考 Franka 官方的夹爪设备接口；Control 客户端和夹爪控制器依赖它。
它不依赖 hardware、control、MoveIt 或 Nav，也不包含运行框架。
`libmfr3duo_hardware` 和纯 CMake `mfr3duo_mujoco` 均不链接该 ROS 接口包。

MoveIt 和 Nav 不因运行时最终使用 control 包中的 controller 而链接 `mfr3duo_control`。
control launch 对 hardware 的启动依赖与 C++ facade 的链接依赖分开声明。

运行时数据流为：

```text
Application / Agent / UI
          │
          ▼
    mfr3duo_robot
      /    |    \
     ▼     ▼     ▼
 moveit   nav   control
     \     |     /
      \    |    /
       ▼   ▼   ▼
    ROS execution layer
          │
          ▼
    mfr3duo_hardware
          │
          ▼
    mfr3duo_mujoco
```

其中实际执行路径为：

```text
MoveGroup
    → MoveIt TrajectoryExecutionManager
    → FollowJointTrajectory
    → arm/spine JTC

Navigator
    → Nav2
    → cmd_vel
    → TmrController

Control facade
    → action / topic
    → JTC / Mfr3DuoGripperController / TmrController

controllers
    → controller_manager
    → Ros2ControlAdapter
    → RobotHardware
    → 同一个 MuJoCo simulation instance
```

任务层基础能力调用的边界为：

> `mfr3duo_robot` 可以调用 `mfr3duo_control` 的基础能力，但只能用于任务编排中确实不属于 MoveIt/Nav 的基础动作，例如夹爪开合。

例如 Pick：

```text
RobotTask
   │
   ├── MoveGroup → arm + spine motion
   │
   ├── Control   → gripper open / close
   │
   └── PlanningSceneInterface
```

仍然禁止：

```text
Robot
    × 直接访问 RobotHardware
    × 直接发送 controller topic
    × 直接访问 hardware_interface

MoveIt
    × 直接访问 RobotHardware
    × 自己控制 gripper GPIO
    × 自己实现 joint controller

Nav
    × 直接控制 steering/wheel joint
    × 实现 TMR IK

Control
    × 做碰撞规划
    × 做导航路径规划
    × 实现 Pick / Place 任务逻辑
```

四层含义保持：

```text
Control:
    怎么执行已经确定
    → 负责执行

MoveIt:
    哪些自由度可运动、目标在哪里
    → 负责联合运动规划

Nav:
    底盘要去哪里
    → 负责导航规划和跟踪

Robot:
    最终要完成什么
    → 负责能力编排
```

### 公共实现规范

命名空间统一：

```cpp
namespace mfr3duo_control
namespace mfr3duo_moveit
namespace mfr3duo_nav
namespace mfr3duo_robot
```

公开长生命周期对象采用 PImpl，并禁止 copy/move：

```cpp
class Xxx
{
public:
    ~Xxx();

    Xxx(const Xxx&) = delete;
    Xxx& operator=(const Xxx&) = delete;

    Xxx(Xxx&&) = delete;
    Xxx& operator=(Xxx&&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
```

析构函数在对应 `.cpp` 中、`Impl` 完整定义后实现。该规则适用于
Control、MoveGroup、Navigator、Robot 和 PlanningSceneInterface。
Plan 是只读值对象，Handle 是共享操作状态的值对象，二者不套用 facade 的禁止移动规则。

库不自行：

```cpp
rclcpp::spin(...)
```

最终应用负责 Executor。

推荐：

```text
MultiThreadedExecutor
+
Task/Application Worker Thread
```

所有需要 action/service callback 的内部 ROS client 使用独立的：

```text
Reentrant CallbackGroup
```

同步接口允许等待结果，但前提是 Executor 正在其他线程运行。

不允许：

```text
SingleThreadedExecutor callback
    ↓
同步等待同一 executor 中 action result
```

这种可能形成自阻塞的调用方式。

### initialize 语义

所有主要 facade：

```text
Control
MoveGroup
Navigator
Robot
```

都提供：

```cpp
Result initialize(
    std::chrono::milliseconds timeout);
```

语义：

```text
第一次成功
    → Ready

已经 Ready 且连接、状态及操作终止状态仍满足条件
    → 直接 Success

失败
    → 保持可重试状态

initialize()
    × 不启动 executor
    × 不无限等待
```

`is_ready()` 必须结合当前依赖可用性判断；`TerminationUnknown` 时返回 false。
Robot 的 `initialize()` 返回 `TaskResult`，其它 facade 返回本包 `Result`；超时使用单一总截止时间。

### 命令完成语义

V1 统一规定：

```cpp
command_arm_joint_position()
command_spine_position()
command_gripper()
execute_arm_trajectory()
move()
execute()
navigate_to()
robot.execute()
```

均为**同步完成语义**：

> 返回 `Success` 表示动作已经执行结束并收到成功结果，而不是仅表示 action goal 被接受。

异步操作统一通过：

```text
start...
TaskHandle
NavigationHandle
```

表示。

唯一例外是连续速度命令：

```cpp
command_base_velocity()
```

其成功仅表示：

> 消息已经在本地 publisher 成功提交。

topic 没有 controller 接收确认。Success 不承诺 TmrController 已收到命令或机器人已运动。
未来需要确认时须增加协议，不能从 publisher 成功推断接收成功。

因为速度控制不存在“达到目标位置后完成”的自然结束状态。

### 超时与取消

轨迹的：

```cpp
duration
```

表示期望轨迹时间，不等同于 API timeout。

例如：

```cpp
command_spine_position(
    0.30,
    1500ms);
```

其中 `1500ms` 表示：

```text
trajectory time_from_start
```

实际 action 等待超时使用配置：

```yaml
execution:
  goal_response_timeout: 2.0
  timeout_margin: 2.0
  cancel_timeout: 1.0
  terminal_timeout: 2.0
  gripper_timeout: 10.0
  navigation_timeout: 120.0
  task_timeout: 180.0
```

这些时间单位为 s，均须有限且为正；示例值在对应 Phase 验收前固定到配置中。
轨迹结果等待上限为：

```text
trajectory_duration
+
timeout_margin
```

规划采用 `allowed_planning_time`，goal response、夹爪、导航、Task 和停止等待分别受对应截止时间约束。
Task 的截止时间覆盖全部子步骤；停止期间仍使用有限的 cancel/terminal 等待，不能因任务超时而丢弃活动子操作。

所有 action 操作采用同一终止状态语义：

```cpp
enum class OperationState
{
    Idle,
    WaitingForGoalResponse,
    Active,
    Canceling,
    TerminationUnknown,
};
```

正常终止结果或 goal 明确拒绝后回到 Idle。下列 CancelGoal 流程适用于 Control/Nav2；MoveGroup 使用本节后文规定的官方 stop 事件及共享通道终止确认。超时和显式取消走以下流程：

```text
request cancel（只针对该操作的 goal ID）
    ↓
wait cancel response
    ↓
wait terminal result
    ↓
CANCELED / SUCCEEDED / ABORTED
    → 确认操作终止，回到 Idle

无法确认 terminal result
    → TerminationUnknown
    → 返回 CancelFailed
    → is_ready() == false
    → 同资源新操作返回 PreviousOperationNotTerminated
```

取消响应成功只表示进入 CANCELING，必须继续确认终止结果；参见
[Humble CancelGoal 定义](https://github.com/ros2/rcl_interfaces/blob/humble/action_msgs/srv/CancelGoal.srv)。
因本地 deadline 触发取消，确认终止后返回 Timeout；显式 cancel 成功后返回 Canceled。
若终止与取消竞争，先确认并保存真实终止结果，不将“已终止”误报为 CancelFailed。

必须覆盖以下竞态：

```text
goal response 超时
    → 标记 local_timeout，保留 pending response 和操作身份
    → 迟到 Accepted：立即 cancel，并确认 terminal result
    → 迟到 Rejected：确认无活动 goal，回到 Idle

cancel 被拒绝 / unknown goal / already terminated
    → 检查该操作的 terminal result
    → 已终止：正常收敛
    → 未知：TerminationUnknown
```

等待截止后仍未确认终止时，返回 CancelFailed 并保持资源阻塞；迟到回调继续执行取消/收敛流程。
禁止丢弃 pending future 后释放资源，或把 TerminationUnknown 当作已经停止。
只有确认终止或显式完成底层停用及恢复，才能重新允许同资源操作。
Robot Task 必须等当前子操作终止后才能启动下一步；子操作状态未知时 Robot 进入 Error。
Control、MoveGroup、Navigator、Robot 遵循相同转换规则和竞态测试，不为此新增 common/runtime 框架包。

每一层的：

```text
stop()
cancel()
```

Control 和 Navigator 只停止**该对象自己拥有的操作**。MoveGroup 采用下述官方停止入口及独占执行通道例外。

例如：

```text
Control::stop_arm()
```

不能承诺取消由 MoveIt 发出的 trajectory。

```text
MoveGroup::stop()
```

使用官方 `/trajectory_execution_event` 发布 `stop`，停止整个 MoveGroup 执行通道。
应用独占并串行使用该通道；同一 ROS Context、共用外部 Node 的 facade 共享执行租约。
禁止其他客户端并行调用 MoveGroup 执行或直接使用相关轨迹控制器。
不承诺按父 goal UUID 隔离其他客户端。

上述 CancelGoal 状态机适用于 Control/Nav2；MoveGroup 无 cancel ACK，确认条件为：
父 ExecuteTrajectory 终态、所有相关 FJT goal 的标准 get_result 终态，以及停止后新鲜关节速度连续
100 ms 静止（手臂 ≤ 0.02 rad/s、spine ≤ 0.003 m/s；反馈年龄 ≤ 300 ms）。
失败或证据缺失时返回 CancelFailed/TerminationUnknown，保留租约并等待迟到确认。
迟到 Accepted 重发官方 stop；原版官方控制器插件和默认执行 capability 保持原样，不保留 compat。

```text
Navigator::cancel()
```

只取消该 Navigator 发起的 Nav2 goal。

```text
Robot::stop()
```

负责依次停止它当前任务拥有的：

```text
MoveGroup
Navigator
Control gripper action
```

`stop()` 不是系统级 Emergency Stop。

真正的 E-Stop 应由未来独立 safety layer 负责。

### 命令所有权

ROS 2 action/controller 本身允许多个 client，因此 V1 明确采用：

> 单 controller 单逻辑 owner 的运行约定，而不是在本阶段实现分布式资源锁。

正常整机运行：

```text
Application
    ↓
mfr3duo_robot
```

是唯一顶层 owner。

下面这些 API：

```text
Control
MoveGroup
Navigator
```

也可以直接用于：

```text
开发
测试
调试
独立示例
```

但调用者必须保证它们不与正在运行的 `RobotTask` 竞争同一资源。

如果未来存在多个独立进程同时控制同一机器人，再增加统一 Resource Lease / Arbitration，不在 V1 提前实现。

### facade 并发、析构与 Handle 生命周期

一个 facade 同时最多一个修改状态或执行命令的操作；第二个执行或配置修改返回 Busy。
只读状态查询线程安全。stop/cancel 必须可从其它线程作用于活动操作，不能等待执行锁而自阻塞。
等待 action/service 时不持有阻止结果回调或取消的 mutex。

析构不抛异常：对活动操作 best-effort cancel，有限等待终止，并停止、join 本对象的 worker。
Executor 由应用持有，必须在 facade 完成清理后再停止。禁止 detach 本地任务线程。
无法确认远端终止时保存 CancelFailed/TerminationUnknown；析构完成不代表远端 goal 已停止。

Handle 只共享操作记录，不引用已经销毁的 facade；操作记录保留最终结果。
owner 正常完成后 Handle 仍可读结果；owner 在活动状态析构时先取消和 join。
未确认终止的清理结果必须可从 Handle 读取，不能报告虚假的 Success。

---

## 2. `mfr3duo_control`：基础控制、夹爪适配与 TMR Controller

`mfr3duo_control` 负责：

```text
arm joint command
arm trajectory execution
spine position
gripper open/close/grasp
base velocity
TMR kinematics
odom
```

不负责：

```text
collision planning
IK goal planning
navigation
robot task
```

### Controller 拓扑

Controller 配置从：

```text
mfr3duo_hardware/config/controllers.yaml
```

迁移到：

```text
mfr3duo_control/config/controllers.yaml
```

目标拓扑：

```text
controller_manager
│
├── joint_state_broadcaster
│
├── imu_broadcaster
│
├── left_arm_controller
│     └── JointTrajectoryController
│
├── right_arm_controller
│     └── JointTrajectoryController
│
├── spine_controller
│     └── JointTrajectoryController
│
├── left_gripper_controller
│     └── Mfr3DuoGripperController
│
├── right_gripper_controller
│     └── Mfr3DuoGripperController
│
└── tmr_controller
      └── TmrController
```

这里修正此前设计：

> V1 不直接使用官方 `GripperActionController`。

原因是当前 hardware contract 不是标准 finger-joint position command，而是：

```text
left_gripper/width
left_gripper/velocity
left_gripper/effort
```

直接增加 YAML 无法解决接口不匹配，也无法正确处理 width 与单指 joint position 的转换，以及 `max_effort`。

因此 V1 保留现有 `RobotHardware` 整机命令模型，在 ros2_control 层新增一个很薄的夹爪适配 controller。

### Gripper hardware / ROS 语义

当前物理语义：

```text
hardware width
    = 两指总开口

single finger position
    = width / 2
```

如果 URDF 单指范围：

```text
0.00 ～ 0.04 m
```

则：

```text
hardware width
    = 0.00 ～ 0.08 m
```

必须只有一个地方实现该换算，禁止：

```text
RobotTask 换算一次
MoveIt 换算一次
Controller 再换算一次
```

统一由：

```text
Mfr3DuoGripperController
```

负责。

标准 action 对外仍使用：

```text
control_msgs/action/GripperCommand
```

规定其：

```text
command.position
```

表示 MoveIt/URDF 中的**单指 joint position**：

```text
0.00 ～ 0.04 m
```

controller 内部转换：

```cpp
hardware_width =
    2.0 * finger_position;
```

状态反向：

```cpp
finger_position =
    hardware_width * 0.5;
```

`max_effort` 不丢弃：

```text
GripperCommand.max_effort
        ↓
Mfr3DuoGripperController
        ↓
gripper effort command interface
```

V1 公共 C++ API 使用 `std::optional<double>`：nullopt 使用 controller default_effort，
显式值必须有限、为正且不超过 hardware 范围，显式 0/负数返回 InvalidArgument。

标准 GripperCommand goal 没有 optional 字段，因此 wire 上 `max_effort == 0` 表示请求 controller 默认值；
Control 仅将 nullopt 编码为这个 wire 值，controller 在写 hardware 前解析为正的 default_effort。
显式非正值不会由 C++ facade 发出。直接 action client 的负数、NaN/Inf 或越界值拒绝。
任何路径都不得把“使用默认值”的 wire 0 直接转发为 hardware 零驱动力。

controller 参数（右侧同样配置）：

```yaml
left_gripper_controller:
  ros__parameters:
    default_velocity: 0.05
    default_effort: 20.0
    min_position: 0.0
    max_position: 0.04
    goal_tolerance: 0.001
    stall_velocity_threshold: 0.001
    stall_timeout: 0.5
    allow_stalling: true
    cancel_hold_effort: 10.0
```

default_velocity 是两指总开口 width 的速度，单位 m/s；0.05 对应单指 0.025 m/s。
position、goal_tolerance 和 stall_velocity_threshold 分别使用单指 m、m、m/s。
effort 是 hardware 驱动 finger 的非负力上限，单位 N，当前范围 0～100；
default_effort 和 cancel_hold_effort 必须有限且为正。
配置速度须满足当前 hardware 总开口速度上限 0.20 m/s；参数不合法时 controller configure 失败。

每次执行都写 width、default_velocity 和解析后的 effort 三个命令值。
达到单指位置容差时 action 成功；未到达目标但持续停滞且 allow_stalling=true 时也可成功，
result 标明 stalled=true、reached_goal=false。allow_stalling=false 时停滞返回执行失败。
停滞判断基于实测速度、未到达目标及力限制状态，持续时间由 controller 的 stall_timeout 决定，
不能直接把后端已有 stalled 的不同计时策略当作同一判据。
这些行为与 [Humble gripper 的完成/停滞语义](https://control.ros.org/humble/doc/ros2_controllers/gripper_controllers/doc/userdoc.html) 对齐。

Gripper Action Success 表示夹爪动作完成；Robot Pick Success 还要求目标物体通过抓取观察验证。
cancel 在 update 中读取当前快照位置，设为 hold target，使用 default_velocity 和 cancel_hold_effort；
确认 hold 命令应用后才产生 CANCELED 结果。cancel 不隐式开爪或把 effort 设为 0。
Task 需要主动松手时显式执行 open 阶段；hold effort 是否足以持物由 Phase 1B 的测试验证。

### Finger joint state

为了：

```text
robot_state_publisher
MoveIt current state
PlanningScene
TF
```

能够获取真实夹爪状态，ROS2 adapter 需要额外提供标准 finger joint state。

V1 固定导出：

```text
left_fr3v2_1_finger_joint1/position
left_fr3v2_1_finger_joint1/velocity

right_fr3v2_1_finger_joint1/position
right_fr3v2_1_finger_joint1/velocity
```

当前 URDF 的第二个 finger 已是 mimic joint。V1 只新增左右手各自驱动 finger 的状态，
由 URDF mimic relationship 推导每只手的另一根 finger。

```cpp
finger_position = gripper_width * 0.5;
finger_velocity = gripper_width_velocity * 0.5;
```

两个值来自同一次 RobotHardware::read_state() 的 coherent snapshot。
controller 做命令换算，adapter 做状态投影，RobotTask 不重复 width/finger 换算。

迁移后的接口数量冻结为：

| 契约 | 当前 | 投影后 |
|---|---:|---:|
| joints | 19 | 21 |
| state interfaces | 70 | 74 |
| command interfaces | 53 | 53 |

新增 joints 只有 state，没有 command；保留 2 GPIO、1 IMU sensor。
必须同步修改 ros2_control Xacro、valid_info()、state storage、export_state_interfaces()、
test fixture、interface count tests、architecture.md 和 README。

原有：

```text
gripper/width
gripper/velocity
gripper/effort
```

GPIO contract 可以继续保留给 RobotHardware / Controller 使用。

因此：

```text
RobotHardware
    不需要为了 MoveIt 改变整机抽象

Ros2ControlAdapter
    增加标准 joint-state projection

Mfr3DuoGripperController
    负责 action ↔ GPIO command 适配
```

这是 V1 的冻结方案。

### Gripper 与联合轨迹

V1 明确：

> Gripper 不属于 arm/spine 的同步 JointTrajectory execution。

因此 `mfr3duo_moveit::RobotGroup` V1 改为：

```cpp
enum class RobotGroup : std::uint8_t
{
    LeftArm,
    RightArm,
    Spine,
};
```

暂时不包含：

```text
LeftGripper
RightGripper
```

原因不是 MoveIt model 中不能存在 gripper，而是：

> Humble 的 GripperCommand execution 只表达最终夹爪目标，不能保证按 RobotTrajectory 中多个时间点与 arm/spine 严格同步。

因此：

```text
Arm + Spine
    → coordinated trajectory

Gripper
    → independent action stage
```

这也与 Pick/Place 的真实任务顺序一致：

```text
open
move
approach
close
lift
```

未来只有在实现了真正 trajectory-capable gripper controller 后，才考虑把 gripper 加入 synchronized `RobotGroup`。

### Arm 与 Spine

双臂继续：

```text
JointTrajectoryController
```

Spine 从当前：

```text
JointGroupPositionController
```

迁移为单关节：

```text
JointTrajectoryController
```

这样 arm/spine 都具有一致的：

```text
FollowJointTrajectory
```

执行模型。

迁移后必须重新验证：

```text
step response
trajectory duration
position error
controller state
joint limits
```

不能只因为 controller 能 activate 就认为迁移完成。

### Control 类型与接口

```cpp
namespace mfr3duo_control
{

enum class Arm : std::uint8_t
{
    Left,
    Right,
};

enum class Gripper : std::uint8_t
{
    Left,
    Right,
};

struct BaseVelocity
{
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

enum class ErrorCode
{
    Success,

    NotInitialized,
    NotReady,
    InvalidArgument,
    StateUnavailable,
    StateStale,
    Busy,
    PreviousOperationNotTerminated,

    ControllerUnavailable,
    GoalRejected,
    ExecutionFailed,

    Timeout,
    Canceled,
    CancelFailed,

    InternalError,
};

struct Result
{
    ErrorCode code{ErrorCode::Success};
    std::string message;

    explicit operator bool() const noexcept
    {
        return code == ErrorCode::Success;
    }
};

template <typename T>
struct StateResult
{
    Result result;
    T value{};

    rclcpp::Time stamp;

    explicit operator bool() const noexcept
    {
        return static_cast<bool>(result);
    }
};

}
```

状态 getter 不再裸返回：

```cpp
double
std::vector<double>
```

因为这无法区分：

```text
当前值恰好为 0
没有收到状态
状态已经过期
```

接口：

```cpp
class Control
{
public:
    explicit Control(
        const rclcpp::Node::SharedPtr& node);

    ~Control();
    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;
    Control(Control&&) = delete;
    Control& operator=(Control&&) = delete;

    Result initialize(
        std::chrono::milliseconds timeout);

    bool is_ready() const;

    Result command_arm_joint_position(
        Arm arm,
        const std::vector<double>& positions,
        std::chrono::milliseconds duration);

    Result execute_arm_trajectory(
        Arm arm,
        const trajectory_msgs::msg::JointTrajectory& trajectory);

    Result stop_arm(
        Arm arm);

    Result command_spine_position(
        double position,
        std::chrono::milliseconds duration);

    Result stop_spine();

    Result command_gripper(
        Gripper gripper,
        double finger_position,
        std::optional<double> max_effort = std::nullopt);

    // 总开口 [m]、速度 [m/s]、力 [N]；定义参考 Franka 官方。
    Result move_gripper(Gripper gripper, double width, double speed);
    Result grasp_gripper(Gripper gripper, double width, double speed, double force,
                        double epsilon_inner = 0.005, double epsilon_outer = 0.005);
    Result stop_gripper(
        Gripper gripper);

    Result command_base_velocity(
        const BaseVelocity& velocity);

    Result stop_base();

    StateResult<std::vector<double>>
    get_arm_joint_positions(
        Arm arm) const;

    StateResult<double>
    get_spine_position() const;

    StateResult<double>
    get_gripper_position(
        Gripper gripper) const; // 旧 API：单指位置。
    StateResult<double> get_gripper_width(Gripper gripper) const; // 两指总开口。

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
```

删除原先容易产生误解的：

```cpp
stop_all();
```

因为 Control 无法可靠停止由 MoveIt/Nav2 其他 client 创建的 goal。

整机停止由：

```cpp
Robot::stop()
```

编排。

### TMR Controller

不再使用：

```text
wheel_base_x
wheel_base_y
```

这种存在“全轴距还是半轴距”歧义的配置。

每个 steer-drive module 显式配置其相对 `base_link` 的有符号安装坐标：

```yaml
tmr_controller:
  ros__parameters:
    command_timeout: 0.5

    modules:
      module_0:
        steering_joint: ...
        drive_joint: ...
        x: ...
        y: ...
        steering_offset: ...
        steering_sign: 1.0
        drive_sign: 1.0

      module_1:
        steering_joint: ...
        drive_joint: ...
        x: ...
        y: ...
        steering_offset: ...
        steering_sign: 1.0
        drive_sign: 1.0

    wheel_radius: ...

    odom_frame: odom
    base_frame: base_link

    publish_odom: true
    publish_tf: true
```

坐标统一：

```text
+x
    base_link forward

+y
    base_link left

+z
    upward
```

对于 module `i`：

```cpp
vx_i =
    vx - wz * y_i;

vy_i =
    vy + wz * x_i;
```

目标方向：

```cpp
angle =
    atan2(vy_i, vx_i);
```

目标轮速：

```cpp
speed =
    hypot(vx_i, vy_i) /
    wheel_radius;
```

然后：

```text
安装 offset/sign 修正
        ↓
连续 steering 最短路径
        ↓
必要时 angle + π
        ↓
drive speed 反向
```

如果 steering 旋转超过 `π/2`，优先：

```text
steering + π
wheel velocity *= -1
```

以减少 steering 转动。

C++17 中禁止：

```cpp
std::numbers::pi
```

统一：

```cpp
constexpr double kPi =
    3.14159265358979323846;
```

零速时：

```text
|vx_i| + |vy_i| < epsilon
```

保持当前 steering angle，不跳回 0。

`±π` tie 使用固定策略，保证同一输入每次结果一致。

TMR Controller 是：

```text
odom -> base_link
```

唯一发布者。

#### TMR 状态路径与 odometry

odom 位姿由实测 steering 与 wheel encoder 增量计算，不积分目标 cmd_vel。
对 module i 先消除安装 offset/sign，再计算小时间区间内的运动：

```text
ds_i = wheel_radius * drive_sign_i * Δwheel_position_i
theta_i = 校正后的区间实测 steering 方向

dx_i = ds_i * cos(theta_i)
dy_i = ds_i * sin(theta_i)

dx_i = dx - dtheta * y_i
dy_i = dy + dtheta * x_i
```

所有 module 共同构造 `A * [dx, dy, dtheta]^T = b`，用 least squares 求底盘位姿增量。
先按关节轨迹连续性解包 encoder delta，再积分到 odom；使用 steering 的区间中点方向，
转向变化过大时细分或降低该区间 confidence。
发布 twist 时以位姿增量除以同一组 ROS timestamp delta。
timestamp delta 非正、reset 跳变或 encoder 数据无效时不继续积分，重新建立基线并报告 diagnostics。

当前 hardware 每次 update 推进固定物理步，仿真可能低于实时速度。
禁止用仿真 joint velocity 乘实际 wall-clock period 计算 odom 位移。
Phase 4 必须验证正常及低于实时速度时 odom 与同实例 ground truth 的一致性。

初始化检查矩阵 rank/condition number；运行时检查 module residual/disagreement。
几何退化时 configure 失败；严重不一致或滑移时降低 odom confidence、增加 covariance 并报告 diagnostics。
禁止静默把无效估计当作正常速度。

#### TMR steering 过渡、实时命令与生命周期

用校正后的实测 steering 计算最短角误差，再限制 wheel velocity：

```text
abs(error) >= steering_stop_threshold
    → wheel velocity = 0

steering_slow_threshold < abs(error) < steering_stop_threshold
    → wheel velocity 按角误差连续缩小

abs(error) <= steering_slow_threshold
    → 正常 wheel velocity
```

threshold 必须有限，满足 `0 <= slow_threshold < stop_threshold <= π/2`，
实际值由 Phase 1A 的切向/横移/旋转转换测试确定。
统一处理目标轮速饱和以保留底盘 twist 比例；配置 steering rate、body acceleration 和 wheel velocity 上限。
±π tie 和恰好 π/2 的反向选择策略固定并单测。

subscription callback 只校验并写入 `realtime_tools::RealtimeBuffer<Command>`；
update 读取含 vx/vy/wz 和 receive timestamp 的完整快照并检查 watchdog。
callback 不直接修改 update state。update 不阻塞等待 ROS、不动态分配；odom/TF 通过实时发布机制输出。
配置 body velocity/acceleration 限制与 Nav2 参数保持一致，NaN/Inf 命令拒绝。

```text
activate
    → drive = 0，steering hold current，清空 stale command，建立 encoder baseline

command timeout
    → drive = 0，steering hold，等待新鲜命令

deactivate
    → drive = 0，steering hold，清空命令

reactivate
    → 不重放旧 cmd_vel，重新建立状态基线
```

超时年龄采用 monotonic receive-time 基准；odom/TF stamp 使用 ROS wall time。
更新 clock 异常不延长 watchdog。停止后的实际轮速和重新激活行为必须纳入回归。

---

## 3. `mfr3duo_moveit`：联合规划、求解器验证与执行规则

### 公共语义

V1 `RobotGroup`：

```cpp
namespace mfr3duo_moveit
{

enum class RobotGroup : std::uint8_t
{
    LeftArm,
    RightArm,
    Spine,
};

}
```

组合仍然动态表达：

```cpp
move_group.add_groups({
    RobotGroup::LeftArm,
    RobotGroup::Spine,
});
```

或者：

```cpp
move_group.add_groups({
    RobotGroup::LeftArm,
    RobotGroup::RightArm,
    RobotGroup::Spine,
});
```

不增加：

```text
DualArm
UpperBody
WholeBody
```

公共 API 继续区分：

```text
参与规划的 group
```

与：

```text
有明确最终目标的 group
```

例如：

```cpp
move_group.add_groups({
    RobotGroup::LeftArm,
    RobotGroup::Spine,
});

move_group.add_pose_target(
    RobotGroup::LeftArm,
    target);
```

语义：

```text
LeftArm
    → Pose goal

Spine
    → participating free DOF
```

### 实现不能简单包装 MoveGroupInterface

内部需要维护：

```cpp
struct MoveGroup::Impl
{
    std::set<RobotGroup> groups;

    std::map<RobotGroup, Target> targets;

    std::vector<
        moveit_msgs::msg::Constraints>
        path_constraints;

    std::optional<
        moveit_msgs::msg::RobotState>
        start_state;

    PlanningOptions options;

    ...
};
```

Target：

```cpp
using Target = std::variant<
    PoseTarget,
    PositionTarget,
    OrientationTarget,
    JointPositionTarget,
    NamedTarget>;
```

同一个 RobotGroup V1 只有一个最终 target。

重复：

```cpp
add_pose_target(
    RobotGroup::LeftArm,
    ...);
```

返回：

```text
TargetAlreadyExists
```

修改必须：

```cpp
remove_target(...)
add_pose_target(...)
```

Target 不允许隐式添加 RobotGroup。

### 同时目标的 `goal_constraints` 组装规则

这是冻结规则。

如果用户设置：

```text
LeftArm
    Pose A

RightArm
    Pose B

Spine
    JointPosition C
```

它们表示：

> 三个条件必须同时满足。

因此必须合并到**同一个**：

```cpp
moveit_msgs::msg::Constraints goal;
```

概念上：

```text
goal
├── left arm position constraint
├── left arm orientation constraint
├── right arm position constraint
├── right arm orientation constraint
└── spine joint constraint
```

然后：

```cpp
request.goal_constraints = {
    goal
};
```

禁止：

```cpp
request.goal_constraints = {
    left_goal,
    right_goal,
    spine_goal,
};
```

因为这表示：

```text
left OR right OR spine
```

而不是：

```text
left AND right AND spine
```

只有未来真正支持：

```text
alternative goal A
OR
alternative goal B
```

时，才使用多个 `goal_constraints` 元素。

### 动态 group 与 inactive joints

内部仍允许：

```text
public RobotGroup set
        ↓
resolver
        ↓
static SRDF planning group
```

例如：

```text
{LeftArm}
    → left_arm

{LeftArm, Spine}
    → left_arm_spine

{LeftArm, RightArm}
    → dual_arm

{LeftArm, RightArm, Spine}
    → dual_arm_spine
```

V1 为三个 RobotGroup 的七种非空组合定义精确 SRDF group：

```text
{LeftArm}                   → left_arm
{RightArm}                  → right_arm
{Spine}                     → spine
{LeftArm, RightArm}          → dual_arm
{LeftArm, Spine}             → left_arm_spine
{RightArm, Spine}            → right_arm_spine
{LeftArm, RightArm, Spine}   → dual_arm_spine
```

是否支持某组的特定目标组合由下面的 V1 支持矩阵判断；SRDF group 存在不代表自由 shared-spine 已通过 Gate。
正常路径不使用 superset。仅在明确启用并验证的 fallback 中使用 superset，否则返回 UnsupportedCombination。

例如 public：

```text
LeftArm + Spine
```

但内部不得因为用了：

```text
upper_body
```

而让 RightArm 移动。

Superset 中未参与 joints 的冻结约束必须覆盖：

> 整条轨迹。

因此使用：

```text
path_constraints
```

中的 joint constraint，而不是只添加最终 goal constraint。

初始建议参数：

```yaml
move_group:
  inactive_joint_tolerance:
    revolute: 0.0001
    prismatic: 0.0001
```

这是初始值，不作为永久物理常数。

联合规划 POC 阶段需要验证该 tolerance 是否会显著降低采样成功率，并据实际结果调整。

规划成功后必须遍历：

```text
trajectory point 0
...
trajectory point N
```

逐点检查未参与 joints：

```text
prismatic / bounded revolute:
    abs(q(t) - q_start) <= inactive_joint_validation_tolerance

continuous revolute:
    abs(remainder(q(t) - q_start, 2 * kPi)) <= tolerance
```

不能只检查：

```text
trajectory start
trajectory end
```

否则中间移动后返回原位也会被误判为正确。

fallback 冻结关节的每个轨迹点 position 均等于 start position，velocity/acceleration 均为 0；
若任一点或 controller 插值行为不能满足冻结规则，整条轨迹拒绝。
保留固定数值表示避免 continuous joint 的 2π 编码跳变，不仅检查模角距离。

### 执行前 start state 检查

从：

```text
plan()
```

到：

```text
execute()
```

之间机器人可能已经被其它 client 移动。

因此执行前必须重新读取当前 state，并验证：

```text
current_state
vs
plan.start_state()
```

默认配置例如：

```yaml
execution:
  start_tolerance:
    arm_revolute: 0.02
    spine_prismatic: 0.005
```

超出后：

```text
ExecutionStartStateMismatch
```

禁止直接执行旧轨迹。

### 联合 IK / Goal Sampling 实施门槛

公共 API 可以先稳定，但以下组合不能在验证前标记为“已支持”。

V1 支持矩阵冻结为：

| 参与组 | spine 语义 | 发布范围 |
|---|---|---|
| LeftArm | 不参与，保持 start state | V1 Required |
| RightArm | 不参与，保持 start state | V1 Required |
| Spine | 必须有显式 joint/named target | V1 Required |
| LeftArm + Spine | 显式目标或参与求解的自由变量 | V1 Required，Gate A |
| RightArm + Spine | 显式目标或参与求解的自由变量 | V1 Required，Gate A |
| LeftArm + RightArm | spine 不参与，保持 start state | V1 Required，Gate B |
| LeftArm + RightArm + Spine | spine 有显式 JointPosition/解析后的 Named target | V1 Required，Gate C |
| LeftArm + RightArm + free Spine | 一个共享自由 spine variable | V1.1，Gate D |

Gate D 未通过或当前部署未启用时，自由 shared-spine 组合返回 UnsupportedCombination。
显式 spine target 在 goal sampling 时固定其最终值，轨迹允许 spine 从 start 值运动到该目标；
不能将“最终值固定”解释为 spine 在整条轨迹中不能运动。
不论哪些组参与，至少存在一个最终 target，否则返回 NoTarget。

#### Gate A：LeftArm + Spine

需要明确建立：

```text
spine joint
    +
left arm 7DoF
```

的完整 kinematic chain。

配置一个真正覆盖：

```text
spine → left TCP
```

的 IK solver。

验收目标必须包含至少一个：

> 单独 LeftArm 无法到达，但 LeftArm + Spine 可以到达的目标。

如果 spine 最终没有产生必要运动，则不能证明该能力成立。

RightArm + Spine 同理。

#### Gate B：DualArm

测试：

```text
LeftArm Pose
+
RightArm Pose
```

同时满足。

固定 spine 时可以分别求左右 arm IK，再合并为一个完整 RobotState。
必须同时验证两侧 pose constraints、joint limits、自碰撞和环境碰撞；
只有联合验证通过的 state 才能进入 goal sampling。
先后求 IK 本身允许，独立结果未经联合验证就作为成功目标禁止。

#### Gate C：DualArm Pose + Spine Joint Target

必须验证：

```text
Left Pose
AND
Right Pose
AND
Spine fixed target
```

能够稳定采样。

目标采样固定 q_spine 为显式目标值，再分别求两臂 IK 并联合验证完整状态。
规划仍在 dual_arm_spine 中包含从当前 spine 到目标 spine 的运动。
失败应按实际诊断区分：

```text
IKFailed
GoalSamplingFailed
PlanningTimeout
PathPlanningFailed
```

而不是全部返回一个模糊的：

```text
PlanningFailed
```

#### Gate D：DualArm + free Shared Spine（V1.1）

两个 arm 必须共享同一个 q_spine。支持 multi-tip/shared-variable 的 kinematics plugin，
或自定义 constraint/goal sampler，共同生成并验证完整目标状态。
不能独立求两条 Arm+Spine IK 后合并两个不同的 spine 值。
此 Gate 属于 V1.1 capability，未通过不阻塞其它 V1 Required 组合发布。

每个 Gate 在开始验收前固定以下 manifest：

```text
测试目标集与可达/不可达构造依据
random seeds 与重复次数
position / orientation / joint tolerance
collision scene 与 start state
SRDF group、IK solver、constraint sampler 和版本
planning timeout
最低成功率及 P95 planning time 上限
inactive joint 最大偏差及失败分类
```

阈值未固定的试验只算探索，不算 Gate 通过。平均 planning time 仅作辅助记录。
有限次数 IK/sampling 没找到解返回 IKFailed、GoalSamplingFailed 或 PlanningTimeout；
NoIKSolution 仅在底层 solver 明确提供该诊断时映射，不能声称数值搜索证明数学上无解。

### Planning Error

增加：

```cpp
enum class ErrorCode
{
    Success,

    NotInitialized,
    NotReady,

    Busy,
    PreviousOperationNotTerminated,
    InvalidGroup,
    DuplicateGroup,
    UnsupportedCombination,
    GroupNotAdded,
    GroupHasTarget,
    NoTarget,

    UnsupportedTarget,
    TargetAlreadyExists,
    InvalidTarget,

    InvalidStartState,
    InvalidConstraint,

    IKFailed,
    NoIKSolution,  // 仅映射 solver 明确给出的诊断
    GoalSamplingFailed,
    PlanningTimeout,
    PathPlanningFailed,

    ExecutionStartStateMismatch,
    ExecutionServerUnavailable,
    ExecutionFailed,
    InvalidPlan,
    IncompleteCartesianPath,

    Timeout,
    Canceled,
    CancelFailed,

    InternalError,
};
```

底层 MoveIt 错误码需要尽可能映射到这里，而不是全部压缩。

### Plan 与 CartesianPath

Plan 为只读值对象，只有 MoveGroup 能写入轨迹与执行元数据：

```cpp
class Plan
{
public:
    Plan() = default;

    bool valid() const noexcept;
    const moveit_msgs::msg::RobotState& start_state() const noexcept;
    const moveit_msgs::msg::RobotTrajectory& trajectory() const noexcept;
    const std::vector<RobotGroup>& groups() const noexcept;
    double planning_time() const noexcept;

private:
    friend class MoveGroup;

    moveit_msgs::msg::RobotState start_state_;
    moveit_msgs::msg::RobotTrajectory trajectory_;
    std::vector<RobotGroup> groups_;
    double planning_time_{0.0};
    bool valid_{false};
    std::optional<double> cartesian_fraction_;
};

struct CartesianPath
{
    Plan plan;
    double fraction{0.0};
};
```

默认 Plan 无效；成功 plan() 或 Cartesian 计算填充新的只读 Plan，可复制/移动供调用者保存。
失败时输出无效 Plan，不能误执行上次残留轨迹。
execute() 仍检查 joint_names 唯一且已知、向量长度匹配、数值有限、关节限制、
time_from_start 非负且严格递增、groups 与轨迹一致、start state 匹配及 inactive 规则。
执行依据 Plan 自带的 groups，不能因当前 builder 配置变化而改写既有 Plan 的含义。

### Cartesian 计算与执行契约

V1 只支持 LeftArm / RightArm Cartesian；spine 与另一只 arm 保持 Cartesian 起始状态。
即使 builder 加入 Spine，也不能通过此 API 隐式启用 Arm+Spine Cartesian。
waypoints 使用 PoseStamped，全部 frame_id 相同且非空；按同一已固定的 start state/TF 快照
统一转换到 MoveIt model frame。拒绝不可解析 frame、混合 frame、非有限 pose 或非法 quaternion。

计算始终启用 avoid_collisions、当前 path constraints 和 jump 检查。
eef_step 使用 m，必须有限且为正。jump 参数从配置读取并在 Phase 3 验收前固定：
relative jump_threshold 是无量纲倍数，absolute revolute/prismatic thresholds 分别使用 rad/m；
三者均须有限且为正，不能将相对阈值误当作关节角度阈值。
几何轨迹经过 time parameterization，应用当前速度/加速度缩放；失败返回错误而不生成可执行 Plan。
同一次计算保存 start state、groups、trajectory、planning time 和私有 cartesian_fraction_。

```cpp
constexpr double kCompletePathTolerance = 1e-6;

CartesianPath path;
auto result = move_group.compute_cartesian_path(
    RobotGroup::LeftArm, waypoints, eef_step, path);

if (result && path.fraction >= 1.0 - kCompletePathTolerance) {
    result = move_group.execute(path.plan);
}
```

Result==Success 仅表示 Cartesian 计算正常完成，fraction 表示完成程度。
partial path 可返回诊断，但 execute() 检查 Plan 内部保存的 fraction，未达到完整阈值返回
IncompleteCartesianPath；修改公开 path.fraction 不能绕过检查。Pick/Place 一律只执行 full path。
start-state/trajectory validation、timeout/cancel 和 controller execution 全部复用 execute(Plan)。
Humble 的计算可能成功返回 partial path，且服务包含时间参数化步骤；参见
[Humble Cartesian 实现](https://github.com/moveit/moveit2/blob/humble/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp)。

### MoveGroup 接口

核心接口保持：

```cpp
class MoveGroup
{
public:
    explicit MoveGroup(
        const rclcpp::Node::SharedPtr& node);

    ~MoveGroup();
    MoveGroup(const MoveGroup&) = delete;
    MoveGroup& operator=(const MoveGroup&) = delete;
    MoveGroup(MoveGroup&&) = delete;
    MoveGroup& operator=(MoveGroup&&) = delete;

    Result initialize(
        std::chrono::milliseconds timeout);

    bool is_ready() const;

    Result add_group(
        RobotGroup group);

    Result add_groups(
        std::initializer_list<RobotGroup> groups);

    Result remove_group(
        RobotGroup group);

    Result clear_groups();

    bool has_group(
        RobotGroup group) const;

    std::vector<RobotGroup>
    get_groups() const;

    Result add_pose_target(
        RobotGroup group,
        const geometry_msgs::msg::PoseStamped& target);

    Result add_position_target(
        RobotGroup group,
        const geometry_msgs::msg::PointStamped& target);

    Result add_orientation_target(
        RobotGroup group,
        const geometry_msgs::msg::QuaternionStamped& target);

    Result add_joint_position_target(
        RobotGroup group,
        double position);

    Result add_joint_position_target(
        RobotGroup group,
        const std::vector<double>& positions);

    Result add_named_target(
        RobotGroup group,
        std::string_view name);

    Result remove_target(
        RobotGroup group);

    Result clear_targets();

    bool has_target(
        RobotGroup group) const;

    Result set_start_state(
        const moveit_msgs::msg::RobotState& state);

    Result set_start_state_to_current_state();

    Result set_goal_joint_tolerance(
        RobotGroup group,
        double tolerance);

    Result set_goal_position_tolerance(
        RobotGroup group,
        double tolerance);

    Result set_goal_orientation_tolerance(
        RobotGroup group,
        double tolerance);

    Result add_path_constraint(
        const moveit_msgs::msg::Constraints& constraint);

    Result clear_path_constraints();

    Result set_planning_pipeline_id(
        std::string_view id);

    Result set_planner_id(
        std::string_view id);

    Result set_planning_time(
        double seconds);

    Result set_num_planning_attempts(
        std::size_t attempts);

    Result set_max_velocity_scaling_factor(
        double factor);

    Result set_max_acceleration_scaling_factor(
        double factor);

    Result plan(
        Plan& plan);

    Result execute(
        const Plan& plan);

    Result move();

    Result stop();

    Result compute_cartesian_path(
        RobotGroup group,
        const std::vector<
            geometry_msgs::msg::PoseStamped>& waypoints,
        double eef_step,
        CartesianPath& path);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
```

### MoveGroup builder 更新规则

add_groups({...}) 是原子操作：已有重复、输入重复或任意非法 group 时全部不修改。
remove_group() 遇到该 group 仍有 target 时返回 GroupHasTarget，要求先 remove_target()。
clear_groups() 同样在存在 target 时拒绝，调用者先 clear_targets()；不隐式丢弃目标。
目标值、容差、路径约束和规划配置在执行期间不可修改，返回 Busy。
多个 path Constraints 合并为一个同时成立的 request.path_constraints，冲突返回 InvalidConstraint。
任何目标都不能隐式添加 group；全部 participating group 均无目标时 plan() 返回 NoTarget。

### Gripper 与 MoveIt 的边界

MoveIt robot model 和 PlanningScene 中仍然包含：

```text
left gripper
right gripper
finger joints
collision geometry
```

但 V1：

```text
MoveGroup coordinated planning
    = LeftArm / RightArm / Spine

Gripper execution
    = Control / GripperCommand action
```

因此 Pick：

```text
Move arm to pre-grasp
        ↓
Cartesian approach
        ↓
Control::command_gripper()
        ↓
validate grasp
        ↓
PlanningScene attach
        ↓
lift
```

不会出现“MoveIt 规划了一条 gripper multi-point trajectory，但 controller 实际只执行最后一点”的隐性语义错误。

---

## 4. `mfr3duo_nav` 与 `mfr3duo_robot`：导航运行条件、任务所有权与 Pick/Place

### Navigator 类型

以下类型位于 mfr3duo_nav::types.hpp，Result 的 bool 语义仅为 code==Success：

```cpp
enum class NavigationState
{
    Idle,
    Running,
    Canceling,
    TerminationUnknown,
    Succeeded,
    Failed,
    Canceled,
};

enum class ErrorCode
{
    Success,
    NotInitialized,
    NotReady,
    Busy,
    PreviousOperationNotTerminated,
    InvalidGoal,
    StateUnavailable,
    StateStale,
    ServerUnavailable,
    GoalRejected,
    NavigationFailed,
    Timeout,
    Canceled,
    CancelFailed,
    InternalError,
};

struct Result
{
    ErrorCode code{ErrorCode::Success};
    std::string message;
    explicit operator bool() const noexcept;
};
```

### NavigationHandle

补全之前未定义的类型。

```cpp
class NavigationHandle
{
public:
    NavigationHandle() = default;

    bool valid() const noexcept;

    NavigationState state() const;

    Result cancel();

    Result wait();

    std::optional<Result> result() const;

private:
    friend class Navigator;

    struct Impl;
    std::shared_ptr<Impl> impl_;
};
```

`Navigator::start_navigate_to()` 返回：

```cpp
NavigationHandle
```

而不是 future。

start 在目标尚未接受时也保留操作身份；拒绝、Busy 或参数错误保存为可读取的结果。
wait() 等待该操作按全局截止时间及终止规则收敛，不能无期限阻塞。
默认/无效 Handle 的 cancel/wait 返回 InvalidGoal。

### Navigator 接口

```cpp
class Navigator
{
public:
    explicit Navigator(
        const rclcpp::Node::SharedPtr& node);

    ~Navigator();
    Navigator(const Navigator&) = delete;
    Navigator& operator=(const Navigator&) = delete;
    Navigator(Navigator&&) = delete;
    Navigator& operator=(Navigator&&) = delete;

    Result initialize(
        std::chrono::milliseconds timeout);

    bool is_ready() const;

    Result set_initial_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped&
            pose);

    Result get_current_pose(
        geometry_msgs::msg::PoseStamped& pose) const;

    Result navigate_to(
        const geometry_msgs::msg::PoseStamped& target);

    Result navigate_through(
        const std::vector<
            geometry_msgs::msg::PoseStamped>& targets);

    NavigationHandle start_navigate_to(
        const geometry_msgs::msg::PoseStamped& target);

    Result cancel();

    NavigationState state() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
```

删除模糊的：

```cpp
stop();
```

导航 action 的标准终止语义就是：

```text
cancel active goal
```

底盘速度归零由 Nav2 controller + TMR watchdog 双重保证。

navigate_to/navigate_through 分别使用 NavigateToPose/NavigateThroughPoses action。
cancel 遵循“取消响应后继续确认终止结果”的公共状态机，不取消其它 client 的 goal。
set_initial_pose() 使用 AMCL 初始位姿 topic，Success 表示本地发布完成；
Navigator Ready 还需确认地图、定位 TF、状态新鲜度及 Nav2 lifecycle ACTIVE。

### Nav2 V1 运行条件

导航 V1 明确使用：

```text
static map
+
AMCL
+
Nav2
```

SLAM 不作为第一阶段验收前置条件。

AMCL 冻结使用全向运动模型：

```yaml
amcl:
  ros__parameters:
    robot_model_type: nav2_amcl::OmniMotionModel
    scan_topic: /sensors/lidar_front/scan
    base_frame_id: base_link
    odom_frame_id: odom
    global_frame_id: map
    use_sim_time: false
```

该插件已在 [Humble AMCL 插件声明](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/plugins.xml) 中提供。
地图文件、障碍物和初始位姿与 MuJoCo 场景一致，V1 定位使用前 LiDAR。

TF 唯一职责：

```text
map -> odom
    AMCL

odom -> base_link
    TmrController

base_link -> robot links
    robot_state_publisher
```

任何其它节点不得重复发布：

```text
odom -> base_link
```

### LiDAR

前后 LiDAR 不要求先做 scan merge。

Nav2 obstacle layer 配置两个 observation source：

```text
front_scan
rear_scan
```

分别消费：

```text
/sensors/lidar_front/scan
/sensors/lidar_rear/scan
```

二者都参与：

```text
marking
clearing
```

需要根据当前真实 frame 名称修改参数，不在代码里写死。

local/global costmap 都配置前后 observation sources；AMCL 只消费前 scan。
costmap 的两个 sources 不意味着 AMCL 同时融合两个 scan，V1 不引入 scan merger。
前 scan 定位视野不足的优化以后单独验证。

ROS topic 连接冻结为：

```text
Nav2 最终 cmd_vel 输出 → /tmr_controller/cmd_vel
TmrController ~/cmd_vel → geometry_msgs/msg/Twist
TmrController ~/odom → /tmr_controller/odom（nav_msgs/msg/Odometry）
Nav2 odom_topic → /tmr_controller/odom
```

若启用 velocity smoother，remap 必须作用于 smoother 的最终输出，不能绕过限速。
所有连接由 launch/参数定义，不在 facade 中硬编码绝对 topic。

### Holonomic local controller

TMR 支持：

```text
vx
vy
wz
```

因此 local controller 必须启用 holonomic motion。

V1 优先采用 Humble 已成熟的 DWB，并显式配置：

```text
min/max_vel_x
min/max_vel_y
max_vel_theta

acc_lim_x
acc_lim_y
acc_lim_theta

decel_lim_x
decel_lim_y
decel_lim_theta

vy_samples > 0
```

禁止沿用 differential-drive 默认参数导致：

```text
vy = 0
```

从而浪费横移能力。

footprint 使用机器人真实底盘外形，不使用随意的：

```text
robot_radius
```

替代。

验收必须观察 DWB 实际输出 non-zero linear.y，并完成横移和对角运动。
NavigateTask 的前置条件是导航姿态：arms stowed、spine navigation height、grippers 位于固定 footprint 内。
姿态目标保存为配置中的 joint/named targets，由 MoveGroup 执行后再启动导航。
V1 导航仅验收该固定姿态；携物导航不纳入严格碰撞验收，直到 carried-object footprint/3D collision 已定义并验证。

### 时间策略

当前模拟传感器时间戳映射到 wall clock，且当前系统没有完整 `/clock` 链路。

因此 V1 明确：

```yaml
use_sim_time: false
```

适用于：

```text
hardware adapter
MoveIt
Nav2
Robot task
```

所有 ROS 节点保持 wall time。

不得出现：

```text
部分节点 use_sim_time=true
部分节点 wall time
```

未来如果 ROS simulation adapter 基于同一个 `mfr3duo_mujoco` 实例发布 `/clock`，再整体切换：

```text
all nodes
    → use_sim_time=true
```

时间策略必须全系统一次性切换。

`mfr3duo_mujoco` 保持纯 CMake、无 ROS 依赖；clock 发布属于 ROS adapter。
controller_update_rate 是唯一控制周期参数，launch 同时设置 controller_manager.update_rate 并推导
RobotHardware control_period=1/controller_update_rate。YAML 默认值不得成为独立可调的第二来源。
对 500/1000 Hz 分别断言周期一致；目标周期不构成 wall-clock 硬实时承诺。

### Robot 类型与公开接口

以下定义属于 mfr3duo_robot，TaskState 与 OperationState 分别描述任务结果和底层终止过程：

```cpp
enum class RobotState { Uninitialized, Ready, Executing, Error };
enum class TaskState { Pending, Running, Succeeded, Failed, Canceled };
enum class Manipulator { Auto, Left, Right };

enum class TaskError
{
    None,
    RobotNotReady,
    RobotBusy,
    PreviousOperationNotTerminated,
    InvalidTask,
    PlanningFailed,
    ExecutionFailed,
    NavigationFailed,
    GraspFailed,
    GraspLost,
    RecoveryRequired,
    Timeout,
    Canceled,
    CancelFailed,
    InternalError,
};

struct TaskResult
{
    TaskState state{TaskState::Pending};
    TaskError error{TaskError::None};
    std::string message;
    explicit operator bool() const noexcept
    {
        return state == TaskState::Succeeded && error == TaskError::None;
    }
};
```

TaskError 保留失败子步骤及底层诊断到 message；初始化成功返回 Succeeded/None，
失败返回 Failed 和对应错误，不把默认 Pending 当作初始化成功。
导航姿态不满足且无法完成收拢时 NavigateTask 不启动 Nav2。

### RobotTask 所有权

异步：

```cpp
Robot::start(const RobotTask&)
```

修改。

V1 使用：

```cpp
TaskHandle start(
    std::unique_ptr<RobotTask> task);
```

显式转移所有权。

调用：

```cpp
auto task =
    std::make_unique<PickTask>("box");

auto handle =
    robot.start(std::move(task));
```

从 `start()` 返回后，任务对象属于 Robot，不再允许调用者修改。

同步：

```cpp
TaskResult execute(
    const RobotTask& task);
```

在进入执行时立即调用：

```cpp
task.clone()
```

形成完整输入快照。

因此基类增加：

```cpp
class RobotTask
{
public:
    virtual ~RobotTask() = default;

    virtual std::unique_ptr<RobotTask>
    clone() const = 0;

    std::string_view name() const noexcept;
};
```

所有派生 Task 都必须实现 deep copy `clone()`。

RobotTask 是可修改但非 thread-safe 的输入对象。clone 期间调用者不得并发修改；
V1 规定 execute(task) 调用期间不并发修改 task。异步转移所有权后不再通过残留指针修改输入。
TaskSequence::clone() 深复制全部子 Task，不能共享可修改子输入。

这样：

```text
异步
    → ownership transfer

同步
    → snapshot
```

任务生命周期没有悬空引用。

### TaskHandle

```cpp
class TaskHandle
{
public:
    bool valid() const noexcept;

    TaskState state() const;

    TaskResult cancel();

    TaskResult wait();

    std::optional<TaskResult>
    result() const;

private:
    friend class Robot;

    struct Impl;
    std::shared_ptr<Impl> impl_;
};
```

### Robot 内部资源

```cpp
struct Robot::Impl
{
    mfr3duo_control::Control control;

    mfr3duo_moveit::MoveGroup move_group;

    mfr3duo_moveit::PlanningSceneInterface
        planning_scene;

    mfr3duo_nav::Navigator navigator;

    std::unique_ptr<GraspObserver> grasp_observer;

    ...
};
```

这是有意允许的。

Robot 使用 Control 只进行：

```text
gripper
必要的 task-level primitive control
```

机械臂/spine 运动仍然必须走 MoveIt。

导航仍然必须走 Navigator。

Robot facade 提供：

```cpp
class Robot
{
public:
    explicit Robot(const rclcpp::Node::SharedPtr& node);
    ~Robot();
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot(Robot&&) = delete;
    Robot& operator=(Robot&&) = delete;

    TaskResult initialize(std::chrono::milliseconds timeout);
    RobotState state() const;
    bool is_ready() const;
    bool is_busy() const;
    TaskResult execute(const RobotTask& task);
    TaskHandle start(std::unique_ptr<RobotTask> task);
    TaskResult cancel();
    TaskResult stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
```

start 的同步输入错误或 Busy 保存在返回 Handle 的结果中，不能吞掉失败。
默认/无效 TaskHandle 的 wait/cancel 返回 InvalidTask。
wait 遵循 Task deadline 和底层终止确认规则；TaskHandle 不负责执行新的 Task。
Robot Error 在物理状态未知或子操作未确认终止时禁止启动新任务。
恢复必须先确认底层终止、重新观察物体、修正 PlanningScene，再通过 initialize() 检查 readiness。

### PlanningSceneInterface 契约

PlanningSceneInterface 属于 mfr3duo_moveit，采用 PImpl、显式析构和公共并发规则。
提供 initialize(timeout)、add_collision_object(s)、remove_collision_object、has_object、
attach_object(id, Arm RobotGroup) 与 detach_object(id)；修改返回 Result，查询返回状态/诊断。
attach link、指定 finger/grasp touch links 从 description/group mapping 解析，业务不传 MuJoCo ID。
scene 更新使用可确认的 apply/query 路径，确认完成后才允许下一规划阶段。
object/tool pose 转换到统一 scene frame，导航后重新获取 TF 和观测，不复用导航前的 base-relative object pose。

### 同实例感知与夹爪设备接口

Robot 内部的 GraspObserver 仅组合标准 object/tool PoseStamped，不定义 ROS 抓取观察消息。
物体 Topic 为 `/perception/objects/<object_id>/pose`，工具 Topic 为
`/perception/tools/{left,right}/pose`，前缀可配置。仿真 adapter 与 Ros2ControlAdapter
共享唯一 RobotHardware/Simulation，发布 simulation_world 下的仿真真值；真实感知需独立提供。
工具位姿按物体采样时刻精确匹配或在 50 ms 内插值，不同 frame 使用该时刻 TF。
缺失、过期、无效或无法匹配的观察返回 Unknown，不能据此打开夹爪或修正场景。

夹爪控制通过 mfr3duo_msgs/Move、mfr3duo_msgs/Grasp，定义参考 Franka 官方 Jazzy
固定提交，字段、单位、顺序和 GraspEpsilon 默认值保持一致。总开口宽度单位 m、速度 m/s、力 N。
Control 提供 move_gripper、grasp_gripper 和 get_gripper_width；保留标准 GripperCommand
及原单指位置 API。所有入口共享一个夹爪执行状态及硬件资源。
Move 受阻不到位失败；Grasp 必须获得符合预期物体宽度容差的稳定夹持反馈。
不新增厂商完整状态、物体接触或任务执行消息。

### Pick/Place 与物理仿真

需要严格区分：

```text
PlanningScene attach
```

和：

```text
MuJoCo 中物体真的被抓住
```

这两件事。

`attach_object()` 只改变 MoveIt 的 planning representation。

它不会自动创建：

```text
MuJoCo weld
constraint
physical attachment
```

因此 V1 Pick 流程调整为：

```text
1. object exists in PlanningScene

2. open gripper

3. move to pre-grasp

4. 进入允许指定 finger/grasp links 接触目标的阶段，执行 full Cartesian approach

5. close gripper

6. 确认 Grasp 成功与实际开口符合物体宽度容差，验证初步夹持

7. 初步夹持确认
       ↓
   attach object to PlanningScene

8. lift

9. observe，验证 lift 后 object 与 gripper 相对位姿稳定

10. success
```

不能：

```text
close gripper
    ↓
立即 PlanningScene attach
    ↓
默认认为抓取成功
```

### Grasp 判据

仿真 V1 需要给测试场景增加：

```text
可抓取 box
导航障碍物
```

抓取判据至少包括：

```text
Grasp 动作成功且实测总开口符合预期物体宽度容差
+
目标物体实际抬升达到要求
+
lift 后 object 与 gripper 相对位姿没有明显漂移
```

仅：

```text
gripper stalled
```

不能证明抓住的是目标物体。

GraspObserver 从同实例观察链获取 object pose 和按采样时刻匹配的 tool pose。
设备夹持确认与 lift 后的实际抬升/相对位姿判据分两个阶段确认。
观察最大年龄、relative translation/orientation drift 和观察窗口沿用已固定的物理验收阈值。
持物恢复需要先前已验证的抬升、新鲜实测开口和稳定相对位姿；仅静止靠近不能证明持物。
释放需打开夹爪、工具撤离后物体仍稳定落位；证据不足返回 Unknown。
RobotTask 输入不暴露 MuJoCo 类型，observer 选择通过 Robot 配置完成。

如果抓取验证失败：

```text
do not attach PlanningScene object

open gripper

retreat if safe

状态已确认且恢复完成 → GraspFailed
状态未知或恢复未完成 → RecoveryRequired，RobotState=Error
```

如果已经 attach 后 lift 或其它执行阶段失败，先停止并确认活动 motion 终止，再重新观察：

```text
object 仍被夹持
    → 保留 PlanningScene attached
    → ExecutionFailed

object 已释放或掉落
    → detach
    → 用新观测位姿恢复 world collision object
    → GraspLost

object 状态未知 / 观察过期 / motion 终止未知
    → RecoveryRequired
    → RobotState=Error
    → 禁止继续自动 Task
```

任务状态与 PlanningScene 必须保持一致。

Place 需要确认正在持有指定 object 和 manipulator，规划 pre-place/full Cartesian approach，
显式 open 后检查落位，执行 full Cartesian retreat 并再次验证物体稳定留在支撑位置，
随后确认释放、detach 并更新 world pose。
未确认释放时保留 attached 或进入 RecoveryRequired，不仅凭 open action success 推断物体已放下。

#### 抓取接触与场景恢复

pre-grasp 前目标 object 与 gripper 保持正常碰撞检查。
进入实际 approach/close 的 grasp stage 前，只临时允许该 object 与指定 finger/grasp links 接触。
其它 links、其它 object 及自碰撞规则继续生效，禁止整体关闭 collision checking。
attach 后将目标作为 attached collision object，以明确的 touch_links 进行 lift/retreat 规划。
失败或取消退出 grasp stage 时恢复临时 AllowedCollisionMatrix 条目；
若仍持物则保留 attached-body 的正确接触规则，不能无条件恢复成 world object。

---

## 5. 工程组织、配置迁移与现有项目兼容

### `mfr3duo_hardware`

继续保留：

```text
RobotHardware
Ros2ControlAdapter
Ros2SensorAdapter
hardware plugin
ros2_control Xacro
```

`ros2_control` Xacro 继续定义：

```text
hardware plugin
command interface
state interface
GPIO interface
```

新增 gripper 标准 finger joint state projection 也应在这里完成。

simulation observation adapter 放在 ROS adapter 库中并共享已有后端，
不把 ROS observation 类型加入 RobotHardware 的三个公共头或 core 库链接依赖。
新增后端只读观察 API 保持 MuJoCo 类型封装；标准 PoseStamped 转换与发布生命周期属于 adapter。

迁移出去：

```text
controllers.yaml
controller spawner launch
controller-specific runtime tests
```

### `mfr3duo_control`

目录：

```text
mfr3duo_control/
├── include/mfr3duo_control/
│   ├── control.hpp
│   ├── types.hpp
│   └── tmr_kinematics.hpp
│
├── src/
│   ├── control.cpp
│   ├── tmr_controller.cpp
│   ├── tmr_kinematics.cpp
│   └── gripper_controller.cpp
│
├── config/
│   └── controllers.yaml
│
├── launch/
│   └── control.launch.py
│
└── mfr3duo_control_plugins.xml
```

TMR IK 放独立纯 C++ 文件：

```cpp
TmrKinematics
```

以便脱离 ROS 测试。

### `mfr3duo_msgs`

```text
mfr3duo_msgs/
├── msg/GraspEpsilon.msg
├── action/Move.action
├── action/Grasp.action
├── package.xml
└── CMakeLists.txt
```

独立 rosidl 包，参考 Franka 官方 Jazzy commit
`6cedf7f1a2ca280c433f643eae697be23eb2a15e`，保留许可证及来源哈希。
Control 的客户端和夹爪控制器依赖它；Robot 通过 Control C++ API 调用。
ROS 类型命名空间为 mfr3duo_msgs，与 franka_msgs 客户端不能直接互通。
只新增实际使用的设备接口，感知优先使用 ROS 标准消息。

### `mfr3duo_moveit`

```text
mfr3duo_moveit/
├── include/mfr3duo_moveit/
│   ├── robot_group.hpp
│   ├── move_group.hpp
│   ├── types.hpp
│   └── planning_scene_interface.hpp
│
├── src/
│   ├── robot_group.cpp
│   ├── move_group.cpp
│   ├── planning_scene_interface.cpp
│   └── planning_group_resolver.cpp
│
├── config/
│   ├── mfr3duo.srdf.xacro
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── ompl_planning.yaml
│   └── moveit_controllers.yaml
│
└── launch/
    └── moveit.launch.py
```

`planning_group_resolver.cpp` 保持内部实现，不新增 public header。

### `mfr3duo_nav`

```text
mfr3duo_nav/
├── include/mfr3duo_nav/
│   ├── navigator.hpp
│   └── types.hpp
│
├── src/
│   └── navigator.cpp
│
├── config/
│   └── nav2_params.yaml
│
└── launch/
    └── navigation.launch.py
```

### `mfr3duo_robot`

```text
mfr3duo_robot/
├── include/mfr3duo_robot/
│   ├── robot.hpp
│   ├── task.hpp
│   ├── task_handle.hpp
│   ├── task_result.hpp
│   ├── pick_task.hpp
│   ├── place_task.hpp
│   ├── navigate_task.hpp
│   └── task_sequence.hpp
│
├── src/
│   ├── robot.cpp
│   ├── pick_task.cpp
│   ├── place_task.cpp
│   ├── navigate_task.cpp
│   ├── grasp_observer.hpp
│   └── simulation_grasp_observer.cpp
│
└── launch/
    └── robot.launch.py
```

仍然不增加：

```text
common/
core/
runtime/
factory/
manager/
framework/
pipeline/
```

等抽象层。

### package.xml / CMake 迁移

新建 `mfr3duo_control` 时必须同步处理：

```text
package.xml

CMakeLists.txt

pluginlib export

controller plugin XML install

config install

launch install

header install

library export

ament dependencies
```

加入夹爪 IDL 生成/导出、感知 Topic 参数、adapter 与 observer client 安装声明；
接口包先于使用它的 Control 客户端和控制器构建，不能使 core 硬件库获得 ROS 链接依赖。

`mfr3duo_hardware` 删除不再属于它的：

```text
controller-specific dependencies
controller YAML install
controller launch install
```

`mfr3duo_robot` 的依赖关系调整为：

```text
mfr3duo_control
mfr3duo_moveit
mfr3duo_nav
标准 geometry_msgs/PoseStamped（观察客户端）
```

不再直接依赖：

```text
mfr3duo_hardware
```

文档同步更新：

```text
README
docs/architecture.md
docs/plan.md
启动说明
运行命令
package dependency diagram
```

### runtime integration 回归测试迁移

现有 hardware runtime integration 中通过 topic 发送：

```text
arm
spine
TMR
```

命令的部分不能简单删除。

迁移后拆成：

```text
mfr3duo_hardware
    hardware interface / state export test

mfr3duo_control
    controller runtime integration test
```

Control runtime integration 必须测试：

```text
action goal accepted

execution result succeeded

action goal rejected

cancel acknowledged

terminal result confirmed after cancel

timeout then cancel

late goal acceptance then cancel

TerminationUnknown blocks new operation

joint state changed correctly

TMR cmd_vel watchdog
```

不能只检查：

```text
topic 能 publish
joint 最后动了
```

---

## 6. 开发顺序、验证门槛与最终验收

整个计划不再按照“把所有 API 都先写完”推进，而是采用：

> Contract → Minimum Prototype → Capability Gate → Public API 完成

### Phase 1A：Control 包迁移、Arm/Spine 与 TMR 命令路径

完成 control package、plugin export、config/launch 安装和现有回归迁移；Arm JTC 保持，Spine 改为单关节 JTC。
TMR 实现 signed geometry、IK、steering 过渡限制、RealtimeBuffer、watchdog 和生命周期停止。
controller_update_rate 同时推导 hardware control_period，验证 500/1000 Hz 配置一致。

验收不启动 MoveIt/Nav2：双臂和 spine 的 action result、轨迹时长、位置误差及 joint limits 正确；
TMR 前进、横移、旋转、零速 steering hold、±π/π/2、反向轮速、超时和重激活正确。
保留 hardware sensor、snapshot/concurrency 和安装包 consumer 回归。

### Phase 1B：夹爪、finger state 与 action 终止确认

完成 Mfr3DuoGripperController 的默认速度、optional effort/wire 默认解析、stall 和 cancel hold 契约。
增加真实 finger joint state projection，冻结 21 joints、74 state、53 command interfaces，同步 fixture、Xacro、README/architecture。
Control trajectory/夹爪 action 首先实现公共 OperationState 状态转换，后续 facade 复用同一语义和场景。

必须验证：

```text
finger position 0.00 / 0.04 ↔ hardware width 0.00 / 0.08
同一 snapshot 的 finger position / velocity 和 mimic TF
nullopt 默认 effort、正 effort 限制、显式 0/负数/NaN/越界拒绝
非零 default_velocity；stall success/failure 与 reached_goal/stalled result
cancel 后 hold 不隐式松手
cancel response 后继续等 terminal result
迟到 Accepted 后立即 cancel
already terminated 与 cancel 的竞态
terminal 未确认 → TerminationUnknown → 同资源新命令拒绝
```

**Gate 1：** 不依赖 MoveIt/Nav2，全部设备正常控制，默认夹爪调用可开合，取消及资源阻塞行为可重复验证。
Gate 1 未通过不进入上层 execution 实现。

### Phase 2A：MoveIt 单组与精确 SRDF groups

配置 description/SRDF、end-effector/TCP mapping、kinematics、joint limits、OMPL、JTC mapping 和 MoveIt launch。
建立七种精确 planning group；先验证 LeftArm、RightArm、Spine 单组及 state/TF 完整性。
此时只使用完成 prototype 所需的最小请求与执行代码，不提前实现完整 MoveGroup facade。

### Phase 2B：V1 Required 联合规划 POC

```text
Gate A：LeftArm + Spine、RightArm + Spine
    → 包含必须借助 spine 才能到达的目标

Gate B：DualArm，spine 保持 start state
    → 左右 IK 可分别计算，再联合 RobotState/constraints/collision/limits 验证

Gate C：DualArm + 显式 spine target
    → goal sampling 使用同一个目标 q_spine
    → 轨迹允许 spine 从当前值运动到目标值
```

每个 Gate 按前文 manifest 固定目标集、seeds、次数、容差、场景、timeout、最低成功率和 P95 上限。
确认结果后固定 resolver、SRDF 和 solver/sampler 配置；不得用一次偶然成功宣称正式支持。

**Gate 2：** V1 Required 支持矩阵全部通过。Gate D 的 DualArm + free Shared Spine 单列为 V1.1，
在未通过或未启用时返回 UnsupportedCombination，不阻塞 Gate 2。
Gate D 需要 prototype 验证 multi-tip 或 custom sampler，不能通过堆 facade API 代替求解验证。

### Phase 3：MoveGroup、只读 Plan 与完整 Cartesian 执行

实现原子 add_groups、显式 remove_target/remove_group、NoTarget、目标/path constraint 合并和支持矩阵检查。
完成只读 Plan、execute defensive validation、start-state 检查及精确组；仅对已验证 fallback 启用 superset freeze。
CartesianPath 包含 Plan，PoseStamped 同 frame，V1 仅单臂且 spine 固定。
开启 collision/path constraints/jump 检查及时间参数化；partial path 仅诊断，不可执行。
MoveGroup 执行和取消遵循 Gate 1 的公共终止语义。

验收包括 builder 原子性、无目标、invalid Plan、逐点及插值冻结、过期 start state、frame 错误、
full/partial Cartesian、fraction 外部修改不能绕过检查、执行失败/超时/取消。

### Phase 4：TMR 正运动学、odom 与全向导航

完成 wheel encoder delta + measured steering 的 least-squares 增量估计、rank/residual/clock reset 诊断，
以及 odom/TF 唯一发布。验证正常和仿真低于实时速度时的 odom 位移、ROS twist 和同实例 ground truth。

配置与场景一致的静态地图、AMCL OmniMotionModel、前 scan 定位、前后 costmap observation sources、
holonomic DWB/vy_samples、速度/加速度、固定导航姿态 footprint、cmd_vel/odom remap 和 Nav2 lifecycle。
Navigator 实现 NavigateToPose/NavigateThroughPoses、deadline、取消终止确认，全部节点 use_sim_time=false。

验收前进、横移、旋转、对角运动、实际 non-zero linear.y、定位连续性、避障、取消与 watchdog。
携物导航暂不计入严格碰撞验收，不用该场景替代固定导航姿态测试。

### Phase 5：PlanningScene、同实例 GraspObserver 与物理 Pick/Place

完成夹爪 Move/Grasp IDL、后端 coherent read、共享现有实例的标准感知位姿 adapter 和 Robot 内部 observer。
保留已冻结的观察年龄、相对位姿漂移和采样窗口，加入可抓取物体及障碍物场景。
验证 object/tool pose 的采样时刻匹配/插值、TF、缺失/过期观察及不阻塞 control update。

完成 scene apply/query 确认、指定 grasp links 的临时接触规则、full Cartesian approach/lift/retreat、
attach/detach 与 Place 释放确认。失败按 still-held / released / unknown 分支恢复；
unknown 导致 RecoveryRequired/Robot Error，禁止猜测 scene 状态继续自动执行。

```text
Level A：mock observer + PlanningScene / 步骤编排
    → 验证软件状态机

Level B：同实例物体/工具位姿、实测夹爪反馈 + physical grasp
    → 验证箱子被拿起、随工具移动并实际放下
```

**Gate 3：** observation 数据链和物理状态恢复通过；Level A 不能标记为物理 Pick Success。
此阶段用最小步骤驱动验证能力，完整 Task facade 在下一阶段完成。

### Phase 6：Robot Task 编排与整机 readiness

第一批只实现 NavigateTask、PickTask、PlaceTask、TaskSequence。
同步 execute(const RobotTask&) 深复制输入，异步 start(unique_ptr<RobotTask>) 转移所有权；
Task 非 thread-safe，调用者遵守输入不可并发修改规则。
Robot 同时只执行一个顶层 Task，使用 Control gripper、MoveGroup、Navigator、PlanningScene 和 GraspObserver。

Sequence 串行执行；任意失败/取消/终止未知均不启动剩余子任务。
确认仍持物时保留 attached；掉落更新 world pose；未知状态进入 Error/RecoveryRequired。
完整验证 Task deadline、RobotBusy、取消传播、clone 深复制、owner 析构与 Handle 结果保留。

启动与 readiness 顺序为：

```text
robot_description
    → ros2_control_node
    → hardware ACTIVE
    → controllers ACTIVE
    → move_group ready
    → Nav2 lifecycle ACTIVE、地图及定位 TF ready
    → GraspObserver ready
    → Robot facade ready
```

launch 可以并行启动进程，但 Robot::initialize() 在总 timeout 内确认以上条件。
只运行一个 controller_manager/hardware simulation instance，MoveIt/Nav launch 不重复启动 hardware。
同一 robot_description/model、topic namespace 和 time policy 贯穿整机 launch。

### 最终验收场景

基础控制：

```cpp
control.command_spine_position(
    0.30,
    1s);
```

表示：

> 已知需要升降到 0.30 m，直接执行。

联合运动：

```cpp
MoveGroup move_group(node);

move_group.add_groups({
    RobotGroup::LeftArm,
    RobotGroup::Spine,
});

move_group.add_pose_target(
    RobotGroup::LeftArm,
    target_pose);

move_group.move();
```

表示：

> LeftArm 必须达到目标，Spine 允许参与求解。

双臂与显式 spine 目标（V1 Required / Gate C）：

```cpp
MoveGroup dual_group(node);

dual_group.add_groups({
    RobotGroup::LeftArm,
    RobotGroup::RightArm,
    RobotGroup::Spine,
});

dual_group.add_pose_target(
    RobotGroup::LeftArm,
    left_pose);

dual_group.add_pose_target(
    RobotGroup::RightArm,
    right_pose);

dual_group.add_joint_position_target(
    RobotGroup::Spine,
    0.30);

dual_group.move();
```

所有示例均假定 facade 已 initialize 成功，并检查每次调用的 Result。
固定 spine 的双臂规划只加入 LeftArm 与 RightArm（Gate B）。
加入 Spine 却不设置其 target 的双臂示例属于 V1.1 / Gate D；
只有通过并启用该 capability 后才允许执行，否则返回 UnsupportedCombination。

导航：

```cpp
navigator.navigate_to(
    target_pose);
```

表示：

> 机器人底盘导航到目标位姿。

任务：

```cpp
PickTask pick("box");

pick.set_grasp_pose(
    grasp_pose);

robot.execute(pick);
```

表示：

> 完成完整抓取任务。

Pick 内部实际是：

```text
MoveIt
    pre-grasp

MoveIt
    approach

Control
    close gripper

Simulation / real sensing
    validate grasp

PlanningScene
    attach

MoveIt
    lift

Simulation / real sensing
    validate relative pose after lift
```

最终四层职责冻结为：

```text
mfr3duo_control
    Command Execution
    + ros2_control integration

mfr3duo_moveit
    Coordinated Motion Planning

mfr3duo_nav
    Holonomic Mobile Navigation

mfr3duo_robot
    Robot Task Orchestration
```

实施阶段最重要的原则不是“尽快把所有接口实现出来”，而是：

```text
hardware contract 先闭合

controller semantics 先闭合

V1 Required 联合规划先验证

execution/cancel semantics 先定义

再扩大 public API
```

各实施阶段分别遵守前置门槛：

```text
Gate 1 / 进入上层 execution 前
    Gripper action ↔ GPIO、finger state、终止确认验证通过

Gate 2 / 完成 MoveGroup facade 前
    V1 Required 支持矩阵的 Gate A/B/C 验证通过

Gate 3 / 宣称物理 Pick/Place 成功前
    同实例观察、真实抓取与物理状态恢复验证通过

Gate D / V1.1 free shared-spine
    独立能力门槛，不阻塞 V1 Required 发布
```

本文冻结的是待实现契约与验收标准，不表示当前代码已经具备对应能力。
只有相应 Gate 通过且留存验证证据后，才能把能力状态改为“已支持”。
