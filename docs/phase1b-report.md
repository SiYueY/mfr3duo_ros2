# Phase 1B / Gate 1 实施报告

结论：PASS。允许进入 Phase 2A；未启动 MoveIt/Nav2，未提前实现上层 execution。

## 实现和文件

- mfr3duo_control/include/mfr3duo_control/control.hpp、control_types.hpp：计划规定的 Control PImpl API、同步完成语义、optional effort 和状态 Result。
- mfr3duo_control/src/control.cpp：应用 Node/Executor 模型，独立 Reentrant CallbackGroup；总初始化截止时间、依赖及 active 检查、状态 freshness、URDF limits；按 goal ID 取消并确认 terminal，迟到 Accepted/Rejected、CancelFailed/TerminationUnknown、资源阻塞和迟到终止恢复。
- mfr3duo_control/src/gripper_controller.hpp/cpp：single finger ↔ total width，非零 default_velocity、wire 0 默认 effort、正 effort 限制、基于测量和力上限的独立 stall 计时，cancel snapshot hold。
- hardware adapter hpp/cpp、ros2_control Xacro、hardware_info.hpp 和 interface/runtime tests：新增左右驱动 finger 的 position/velocity，来自同一 RobotHardware snapshot；21 joints、74 state、53 command，2 GPIO 和 1 IMU 不变。
- Control CMake/package.xml/plugin XML/controllers.yaml/control.launch.py/execution.yaml：安装及 export、两侧 gripper action 配置、冻结的执行截止时间。
- gripper_controller_test.cpp、control_action_test.cpp、control_runtime_client.cpp、runtime_integration.py、installed_consumer_test.py：控制器和 facade 测试。
- README.md、architecture.md：接口契约、action/状态和析构顺序同步。

## 相邻项目修复

romujoco 的原 PD 控制在改用实测 width 后会消除夹紧力，不能可靠保持已有负载。
GripperComponent 在目标切换为当前 measured width 时立即重置旧运动参考、保留 closing preload，并始终按新的 effort 上限裁剪。
显式不同 width（包括 open）、零 effort 或 reset 清除预载，公共数据结构不变。

新增 gripper_hold_test.cpp：50 g 自由物体，采集阶段暂时补偿物体重力，hold 阶段移除支持，在重力下保持 2 秒；开口偏移不超过 2 mm、物体位移不超过 5 mm，10 N 上限及更低上限生效，显式 open 释放。
该场景设置明确的接触刚度；结论仅针对该物理 fixture，不是任意物体的保证。标准 gripper command 成功也不等于 Robot Pick 成功。

后端 logging_default_bootstrap 的工作目录隔离，修复并行 ctest 的 simulate.log 串扰。

## Humble 生命周期修复

本机重复测试曾在 facade 析构后、应用 Executor 仍运行时崩溃，gdb 栈位于 rmw_wait。
Humble memory strategy 对 callback-group notify guard 使用 raw pointer，相关上游问题为
[rclcpp #2664](https://github.com/ros2/rclcpp/issues/2664)。
Control 让 Context 保留 callback group 的空壳直到 shutdown；客户端、订阅和 facade 状态正常释放，回调只持有操作记录或 weak state，未引用已销毁 facade。
应用顺序保持 facade 清理 → Executor 停止/join → Context shutdown；不创建内部 executor、不 detach，也不无限等待。

## 验证

- 全部 7 包重新构建成功。
- 最新完整整仓测试：33 tests、0 errors、0 failures、0 skipped；随后新增的 installed consumer 单项通过。
- 后端全部 39 tests 通过，包括 loaded hold、snapshot recycling、并发、传感器和 package consumer。
- 实测 500/1000 Hz：8 个 controllers ACTIVE；两臂和 spine FJT 成功、轨迹时长和限位回归；左右夹爪 0/.04 单指开合、wire 默认/显式正 effort、无效 goal 拒绝、取消 terminal/hold、驱动 finger joint state 和两指 mimic TF。
- 两种控制率下独立 C++ Control facade 实际初始化及 arm/spine/gripper/state/velocity 调用成功。
- Gripper unit：stall success/failure、reached/stalled result、取消 hold、重新激活和 update 零 C++ new。
- Action unit：Busy、取消响应后继续等 terminal、cancel 拒绝/已终止竞争、迟到 Accepted 立即取消、迟到 Rejected、TerminationUnknown、同资源拒绝、late terminal 恢复、状态缺失/过期/恢复以及 GPIO NaN 不污染真实 joint state。
- 生命周期修复后 action unit 连续 30 次成功，之后新增 freshness 测试和最新完整整仓测试也通过。
- 安装包 consumer 只 find_package(mfr3duo_control) 并链接导出 target，编译、链接和运行通过；发现并修复了 facade target 的依赖 export 缺漏。
- git diff --check 通过；docs/plan.md 用户暂存 3427 行未修改。

日志：/tmp/mfr3duo-phase1b-final-build.log、/tmp/mfr3duo-phase1b-final-tests.log、/tmp/mfr3duo-phase1b-final-results.log、/tmp/mfr3duo-phase1b-group-lifetime-tests.log、/tmp/mfr3duo-phase1b-backend-final-tests.log、/tmp/mfr3duo-phase1b-consumer-tests.log。

## Gate 1 边界

Control 已验证命令执行与终止确认，不包含碰撞规划或导航。调用者保证单 controller 单逻辑 owner；stop 不能取消外部 MoveIt/Nav2 client 的 goal。
连续速度成功只表示本地 publisher 提交。TMR odom 在 Phase 4 实现；MoveIt/Nav/Robot 尚为 skeleton，按后续 Phase 推进。
