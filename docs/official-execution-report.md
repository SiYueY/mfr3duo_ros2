# 恢复官方 MoveIt 执行入口

日期：2026-10-04。环境：ROS 2 Humble、MoveIt 2.5.9。

删除 `mfr3duo_moveit/compat`、自定义控制器 manager/handle、TerminalExecuteTrajectory capability、插件 XML 和对应构建/安装规则。生产启动恢复 `moveit_simple_controller_manager/MoveItSimpleControllerManager` 和默认 `move_group/MoveGroupExecuteTrajectoryAction`，`allow_trajectory_execution=true`。未修改 `/opt/ros/humble` 中的框架或插件。此前的兼容层不能整体认定为官方插件 BUG 的修复，原版核查见 [upstream-execution-audit.md](upstream-execution-audit.md)。

## 应用执行契约

MoveGroup facade 发出标准 ExecuteTrajectory goal；stop/本地超时通过官方 `/trajectory_execution_event` 的 `std_msgs/String("stop")` 停止执行。该事件作用于整个 MoveGroup 执行通道。应用独占并串行使用这一通道；同一 ROS Context 的 facade 共用一个持续运行的外部 Node/Executor 和执行租约。其他进程/Context 的调用不能由此租约隔离，禁止其他客户端同时执行 MoveGroup 或直接占用相关 FJT 控制器。Control/Nav2 的按 goal UUID 取消契约不变。

应用层没有替代框架插件。私有 `src/execution_monitor.hpp` 使用标准 Action status 和 get_result，记录派发前的 UUID 集合及派发时间，收集相关控制器派发后的新 goal 结果。只有父 Action 和全部相关子 Action 都确认终态，才能释放执行租约；成功还要求子结果成功。父结果提前返回不表示所有执行资源已释放。

停止额外要求相关关节速度为已知，新采样时间和接收时间均晚于停止请求，采样/接收年龄均不超过 300 ms；手臂速度不超过 0.02 rad/s、spine 不超过 0.003 m/s，并连续满足 100 ms。is_ready 同时检查共享通道。等待截止后仍不能确认，则返回 CancelFailed/TerminationUnknown、阻止新执行；迟到 Accepted 重发官方 stop，迟到结果和反馈继续收敛。对象析构时保留必要通信对象至确认终止或 Context shutdown。

如果官方入口在派发控制器前终止、缺少预期子 goal 或结果无法获取，应用保守保留未知状态，不把父失败结果当作硬件已停止的证明。应用应诊断配置/控制器并完成受控恢复，不能靠重建 facade 绕过未知租约。

测试软件 fixture 是唯一关闭默认 ExecuteTrajectory capability 的启动分支，用于注入迟到响应等竞态；生产启动和多控制器测试均使用官方默认入口。历史 Phase 2B/3 报告已标注旧契约被替代。

## 验证

- Humble 构建：重建至 mfr3duo_robot 的 8 个包成功；最后的 MoveIt/Robot 增量重建成功。旧 MoveIt 包 build/install 清理后重建，已确认替代插件库和注册资源不存在。
- 软件契约：builder、installed consumer、配置、迟到接受/拒绝、本地超时、共享通道 Busy、未知终止、对象析构后收敛通过。多控制器 fixture 检查首个子结果/提前父结果均不能释放通道，全部子结果后仍运动也不能释放，静止确认后下一次执行成功。
- 动态库来源：运行中的 move_group 的 /proc/PID/maps 确认 controller manager 和默认 capabilities 库来自 /opt/ros/humble，未加载原项目替代插件。
- MuJoCo：场景服务、单组执行、facade 联合/Cartesian 实际执行与取消通过。可视化 Navigate → Pick → 搬运 → Place 完成，任务 48.492 s；物体真实抬升 0.099725 m，最终场景对齐误差 0.000902 m。物理夹持/放置及撤离验证通过。
- 完整联合验收：8 组目标 × 10 次，共 80/80 采样/规划通过；实际联合执行、官方停止及后续执行通过，完整测试 163.36 s。该测试也再次核对了官方动态库来源。
- Robot 异常恢复：147.76 s 通过，覆盖 Executor 中断导致未确认终止时禁止新任务、迟到子结果/新鲜静止后显式恢复、持物感知丢失、任务 deadline、实际掉落、活动 Robot 析构及 Handle 结果保留。
- 本轮最新结果：MoveIt 的 9 个 CTest、Robot 的 7 个 CTest，合计 16/16 通过；失败项修正后单独重跑，未将旧失败结果当作通过。SDK/Control 的既有验证不作为本轮新运行结果。

首轮回归修正了 is_ready 未检查共享执行租约的问题。Robot 的中断恢复测试改为等待共享通道实际终止，而非仅等待顶层任务不 Busy；这保留了未确认终止时拒绝恢复/新任务的行为。验证脚本的官方库名和 launch 进程编号匹配亦已修正。

真实机器人验证尚未执行；MuJoCo 结果不能作为真实机器验证。

复核日志位于 `/tmp/mfr3duo-official-build.log`、`/tmp/mfr3duo-official-final-build.log`、`/tmp/mfr3duo-official-contract-final.log`、`/tmp/mfr3duo-official-stop.log`、`/tmp/mfr3duo-official-runtime.log`、`/tmp/mfr3duo-official-gate2.log`、`/tmp/mfr3duo-official-robot.log`、`/tmp/mfr3duo-official-recovery.log`。单独重跑日志对应各组中修正后的失败项。整机可视化指标已摘录在本报告；测试成功后临时 launch 日志会自动删除。

本轮确认的是运行中的执行和停止闭环，不将测试清理时向外部 move_group 发送 SIGINT 视为其析构路径已验收；原版审计记录的关闭异常不在本轮框架修改范围内。
