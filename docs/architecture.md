# 仓库架构

`mfr3duo_ros2` 是 colcon 集成仓库，目标平台为 Ubuntu 22.04 和 ROS 2 Humble。

```text
mfr3duo_description (ament_cmake, submodule)
    ├── mfr3duo_mujoco (纯 CMake, submodule)
    │       └── mfr3duo_hardware (ament_cmake)
    ├── mfr3duo_moveit (ament_cmake)
    └── mfr3duo_nav (ament_cmake)

mfr3duo_control (ament_cmake)
    └── hardware (runtime) + ros2_control interfaces + mfr3duo_msgs (build)

mfr3duo_robot (ament_cmake)
    └── control + moveit + nav + standard PoseStamped perception

mfr3duo_msgs (ament_cmake, rosidl)
    └── Move.action + Grasp.action + GraspEpsilon.msg (Franka Jazzy definitions)
```

`mfr3duo_description` 提供机器人模型和资源。`mfr3duo_mujoco` 保持独立纯 CMake 包，依赖外部安装的 `romujoco`，通过 `colcon.pkg` 声明对 description 的构建依赖，并仅在 colcon 构建中关闭 teleop。colcon 通过安装前缀发现依赖，不使用相邻 submodule 目录的 CMake 路径或 `add_subdirectory()`。

`mfr3duo_hardware` 分为两层。`libmfr3duo_hardware.so` 提供 `RobotHardware`，这是 MFR3Duo 唯一正式的整机硬件 C++ API，封装 `initialize/activate/deactivate/shutdown/update/write_command/read_state`，只私有依赖 `mfr3duo_mujoco`，Public Header 不包含 ROS 或 MuJoCo 类型。`libmfr3duo_ros2_adapter.so` 提供 `Ros2ControlAdapter`（`hardware_interface::SystemInterface`，plugin 名 `mfr3duo_hardware/Ros2ControlAdapter`）与 `Ros2SensorAdapter`，二者共享同一个 `RobotHardware` 实例，依赖方向固定为 `mfr3duo_ros2_adapter → mfr3duo_hardware → mfr3duo_mujoco`。

ros2_control 契约为 21 个 joints、53 个 command interface、74 个 state interface，覆盖双臂、脊柱、TMR 主动关节、夹爪与 IMU。关节状态与 IMU 使用标准 broadcaster，双臂和 spine 轨迹控制使用 JointTrajectoryController；TMR body twist 使用 Control 包中的 TmrController；控制器配置与启动入口迁移到 `mfr3duo_control/config/controllers.yaml` 和 `mfr3duo_control/launch/control.launch.py`。控制周期由 launch 从 `controller_update_rate` 推导为硬件参数 `control_period`。Camera 与 LiDAR 由 `Ros2SensorAdapter` 从同一个 `RobotHardware` 经独立线程发布，不进入控制循环的标量状态接口。`mfr3duo_moveit` 提供精确 SRDF、联合规划、MoveGroup 和只读 Plan；`mfr3duo_nav` 提供 Nav2 全向导航配置及 Navigator。`mfr3duo_robot` 提供 Robot、Task 和 TaskHandle，只编排下层公开接口。

父仓库的 gitlink 固定两个 submodule 的准确提交；`.gitmodules` 的 `develop` 仅用于显式执行 `git submodule update --remote` 时选取上游分支。

Control 包提供纯 C++17 TmrKinematics/TmrOdometry、controller plugins 和 Control facade。
Phase 4 的 odom 使用实测轮编码器增量与转向角；TMR controller 唯一发布 odom→base_link TF。夹爪 action 保留 GPIO width/velocity/effort 命令，
单指位置由 controller 转换为总开口；adapter 从同一 motion snapshot 投影左右驱动 finger 的 position/velocity，
第二根 finger 通过 URDF mimic 推导，不新增 command interface。
Nav 以标准 ROS action 与 Nav2 通信，不编译链接 Control、MoveIt 或 Hardware。
robot 通过 Control、MoveIt、Nav 和观察 IDL 工作，不直接链接 Hardware 或 MuJoCo。
Phase 1A 已通过回归。后端在初始化时准备相机快照的发布/写入存储，回收未发布快照的嵌套引用，
保留读者持有快照的不可变性；验证结果见 [实施报告](phase1a-report.md)。

Control 使用应用传入的 Node，不启动 Executor；同步命令等待终止结果，stop 只取消本对象拥有的 goal。
内部 action/service clients 使用 Reentrant CallbackGroup。取消响应不等于终止；goal response 超时后保留身份，迟到 Accepted 立即取消，
终止未确认时返回 CancelFailed、is_ready 为 false，同资源新命令返回 PreviousOperationNotTerminated。
状态区分未收到与过期，忽略 broadcaster 的非关节 GPIO/IMU 项；initialize 在单一总截止时间内检查控制器 active、action servers、状态和 URDF limits。

Humble callback-group guard condition 在 executor wait set 中使用 raw pointer，facade 先于 executor 析构时可能触发
[rclcpp 上游问题 #2664](https://github.com/ros2/rclcpp/issues/2664)。Control 由 Context 保留 callback group 的空壳直到 shutdown，
其通信对象和状态仍随 facade 释放；应用顺序为 facade 清理 → Executor 停止/join → Context shutdown。

后端当前开口 hold 会保留已有 closing preload，并受新的 effort 上限限制；显式不同宽度目标、零 effort 或 reset 清除预载。
持物验证限定于 50 g、明确接触刚度的平行夹爪测试场景，不能推断任意物体都可由 10 N hold 保持。

Phase 1B / Gate 1 已通过，实际测试范围和边界见 [Gate 1 实施报告](phase1b-report.md)。

Phase 2A 配置七个精确规划 group；arm+spine 使用以 `franka_spine` 为根的真实八自由度单 TCP chain。
单组运行 POC 经 MoveIt TEM 和标准 FollowJointTrajectory action 执行，不链接 Control C++ facade。
MoveIt launch 可包含一次 Control bringup，也可通过 `start_control=false` 复用已运行的硬件。
默认 `use_sim_time=false`。Phase 3 已实现公开 MoveGroup facade，并通过完整回归。

被动 caster/rocker 的五个位置和速度沿既有 `Simulation::read_state(TmrPassiveState&)` 读取，
经 `RobotHardware::read_state(PassiveJointStates&)` 转发，由 SensorAdapter 在控制循环外发布。
它们不进入整机命令、运动 RobotState 或 ros2_control 的 21/74/53 合同。
受控和被动关节使用不重叠的 JointState 消息，MoveIt CSM 合并并检查完整状态新鲜度；
Control 忽略不含受控关节的消息，避免覆盖运动状态或刷新其过期时间。

传感器和相机支架的 collision 几何沿用已有 visual 几何。SRDF 仅排除机械相邻/刚性组件及
明确列出的轮壳、闭合指尖自接触；其余 self/world collision 保持启用。
相机支架位置仍为 description 中注明的暂定安装数据；Phase 5 增加了显式物理 table/box 场景及对应 PlanningScene，同实例观察用于确认实际抓取与释放。

Phase 2B / Gate 2 已通过，见 [验收报告](phase2b-report.md)。Gate A 的高位目标超过
固定 arm 的保守可达上界；Gate B/C 的双侧 IK 合并后统一验证 AND、完整状态和 collision。
MoveIt 使用原版 `MoveItSimpleControllerManager` 和默认 `ExecuteTrajectory` capability，
执行仍通过标准 FJT Action。已删除 compat 与自定义执行 capability，不修改系统 MoveIt。
MoveGroup facade 使用官方 `trajectory_execution_event=stop` 路径停止；该事件作用于整个执行通道，
应用必须独占并串行使用这一通道，禁止其他客户端并行执行或直接占用相同控制器。
同一 ROS Context 的 facade 共享执行租约，使用同一个持续被 Executor 驱动的 Node。
应用通过标准 Action status/result 按新 goal UUID 收集所选控制器终态；父 Action 结果不能单独释放租约。
取消还要求全部相关关节有新鲜速度反馈，并连续 100 ms 满足静止阈值：手臂 0.02 rad/s、spine 0.003 m/s。
缺失子结果或静止证据时保留 TerminationUnknown，迟到结果继续收敛。见 [官方执行迁移报告](official-execution-report.md)。
Gate D 的双臂自由共享 spine 仍不支持。

Phase 4 已通过完整回归，见 [实施报告](phase4-report.md)。同实例 world pose 与 simulation TimeReference
仅作验证；编码器里程计和 AMCL 不使用真值。静态地图与 navigation.xml 一致；AMCL 使用 OmniMotionModel，
两个 costmap 观察前后 LiDAR；holonomic DWB 输出经 velocity smoother 到 TMR。固定导航 footprint
包含 transport 姿态下的上身与双臂，应用必须先通过 MoveGroup 到达配置姿态。携物及任意伸臂姿态
不在当前严格碰撞验收范围内。Navigator 以外部 Node 检查八个 lifecycle、地图、odom 和 map TF；
取消 ACK 不当作终止，未知终止状态禁止新操作，迟到结果可收敛。

当前标准 PoseStamped 感知 Topic 复用 SensorAdapter 所持的 RobotHardware，以不可变 snapshot
发布目标物体与 TCP 位姿，使用采样时间戳及 world frame。Robot 内部观察客户端
使用外部 Node/Executor，检查响应 deadline、采样年龄和有限有效位姿。PlanningSceneInterface
通过 apply/query 确认，临时接触只允许目标物体与选定两指；attached touch rules 随释放确认
后恢复。物理 Pick/Place 与 still-held/released/unknown 恢复由私有步骤驱动验证。
Phase 5 的物理观察与恢复验证见 [Phase 5 验证报告](phase5-report.md)。

Phase 6 已实现 Robot、Navigate/Pick/Place 和串行 TaskSequence；同步 execute 深复制，异步 start 转移所有权，
一个 Robot 同时只有一个顶层任务。外部应用提供并运行多线程 Executor，facade 不拥有线程。
Robot::initialize 依次确认 Control、MoveIt/scene、Nav2 lifecycle/map/TF 与同实例新鲜观察；
robot.launch.py 只启动一个 controller_manager/Hardware/MuJoCo。未知终止或未知持物禁止新任务，
需要实际终止与 fresh observation/scene 修复后显式 initialize；Handle 保留最终结果。
MoveIt 场景使用 base_link，Pick 前从同一新鲜物理观察批量刷新桌面与物体，Place 前刷新桌面，
避免导航后保留过期的基座相对支撑位置；附着前再次同步刷新。桌面的规划几何使用固定 0.1 mm
表面接触余量，处理真实软接触的微米级压入，物理几何和抓取验收阈值不变。
整机测试核对释放物体与支撑桌在最终场景中的相对位置及固定规划桌面尺寸。

双臂位置模式启用后端已有纯重力补偿，避免以实测位置作为下一条轨迹首点时再次下沉。
SDK 使用预分配 scratch data 的 kinematics/CoM/RNE 计算，避免每个关节重复完整碰撞/约束求解。
夹爪稳定阻挡窗口还要求实测闭合力稳定，max_effort 仍是上限。
完整整机物理任务与生命周期验收见 [Phase 6 报告](phase6-report.md)。

## 夹爪接口与感知迁移

mfr3duo_msgs 参考 Franka 官方 Jazzy 固定提交中的三个夹爪定义，独立命名空间，
不依赖 franka_msgs。Control 提供总开口 Move/Grasp，保留 GripperCommand 兼容入口，
所有入口共享硬件资源。JointState 提供实际关节开口；Action 结果不证明物体任务成功。

物体和工具使用 `/perception/objects/<id>/pose`、`/perception/tools/{left,right}/pose`，
在物体采样时刻匹配工具位姿并转换 TF。仿真来源是现有同一实例的真值，非相机识别。
Robot 验证夹持宽度、实际抬升、持物稳定和撤离后的落位；缺少证据进入 Unknown。
历史 Phase 5/6 的服务和双指接触验收记录是迁移前结果，见接口迁移报告。
