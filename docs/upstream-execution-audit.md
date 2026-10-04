# 原版 MoveIt 执行入口核查

> 接口迁移说明：下文的自定义 ObserveGrasp 服务和双指接触契约为历史实现。
> 当前使用 mfr3duo_msgs 夹爪动作及标准物体/工具位姿 Topic，见 [迁移报告](msgs-migration-report.md)。


> 后续实施：compat 和自定义执行 capability 已删除，生产执行恢复官方插件及默认入口。
> 应用采用独占通道及子结果/实测停止确认；见 [迁移报告](official-execution-report.md)。

日期：2026-10-04。核查已完成；不是整机回归通过报告。

## 官方部署依据

核对 Franka 官方仓库 **humble 分支**，而不是当前默认的 jazzy 分支：

- [moveit.launch.py](https://github.com/frankarobotics/franka_ros2/blob/humble/franka_fr3_moveit_config/launch/moveit.launch.py)
  分别启动 `move_group`、`ros2_control_node`，加载原版
  `moveit_simple_controller_manager/MoveItSimpleControllerManager`。
- [fr3_controllers.yaml](https://github.com/frankarobotics/franka_ros2/blob/humble/franka_fr3_moveit_config/config/fr3_controllers.yaml)
  使用 `FollowJointTrajectory` 和 `GripperCommand` Action。
- [FrankaRobotStateBroadcaster](https://github.com/frankarobotics/franka_ros2/blob/humble/franka_robot_state_broadcaster/src/franka_robot_state_broadcaster.cpp)
  从控制层读取状态，发布标准消息及 `franka_msgs/FrankaRobotState`。

同仓库组件之间使用 ROS 2 通信是该部署的正常做法。控制循环里的控制器到
硬件插件通过 ros2_control 的 C++ 状态/命令接口访问；独立应用或 MoveIt
通过 ROS 2 Action/Topic 访问控制程序。这两种边界不可混为一谈。

当前本项目 `task_demo` 内的 `Robot` 是 C++ 库对象。其观察客户端与
`ros2_control_node` 内的 `Ros2SensorAdapter` 通过 `ObserveGrasp` 通信。
`mfr3duo_interfaces` 是内部观察协议，不是 Robot 的上层任务 API。
上层 C++ 应用链接 Robot 库；当前没有跨进程 Robot 任务服务端。

## 最小复现

```bash
cd /home/siyuey/workspace/mfr3duo/mfr3duo_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=195  # 使用未被其他 ROS 程序占用的域
export ROS_LOG_DIR=/tmp/mfr3duo-upstream-audit-logs
python3 mfr3duo_moveit/test/upstream_execution_audit.py \
  --output /tmp/mfr3duo-upstream-audit-final.json
```

测试启动 `/opt/ros/humble/lib/moveit_ros_move_group/move_group`，恢复
`allow_trajectory_execution=true` 和默认 capability，不加载项目的 facade、
controller plugin、capability、Hardware 或 MuJoCo。通过 `/proc/<pid>/maps`
确认实际加载 `/opt/ros/humble/lib/libmoveit_simple_controller_manager.so.2.5.9`，
并检查两个项目兼容插件均未加载。安装的 MoveGroup 和插件版本都是 2.5.9。

三个软件 FJT 服务发布有效 home joint state。轨迹有两个点和非零持续时间，
保持 home 位置；用于检验执行协议，不证明任何真实运动或实际停止。
关闭执行时长监控，避免 watchdog 取消被误认作客户端取消。

最终核查结果：

| 用例 | 实测结果 |
| --- | --- |
| 正常执行 | 三个子目标完成；父状态 SUCCEEDED，MoveIt SUCCESS |
| 直接取消 ExecuteTrajectory Action | 响应延迟约 2.038 秒；取消返回 GOAL_TERMINATED；0 个子目标收到取消；父状态 SUCCEEDED |
| 官方停止事件 | 三个子目标均收到取消；约 0.162 秒父状态 ABORTED，MoveIt PREEMPTED |
| 停止事件，子目标错开结束 | 三个子目标均收到取消；约 0.169 秒返回父结果，此时仅 1 个子目标进入终态 |
| 等所有子目标结束后再次正常执行 | 成功，无需修改官方控制器插件 |

软件控制器正常完成时间为约 2 秒。普通停止用例收到取消后延迟约 150 毫秒
结束；错开用例三个服务分别延迟 150/350/550 毫秒。脚本在继续下一用例前
等待所有软件子目标进入终态，因此停止后的成功不代表可以忽略迟到子结果。

SIGINT 清理原版 MoveGroup 时实测退出码 **-11**，日志末尾在 rclcpp
CallbackGroup 析构中出现 segmentation fault。此退出故障独立记录，未归因于
控制器插件，未修改系统库。脚本返回成功只表示采集到完整执行核查数据，
不表示原版 MoveGroup 正常退出；JSON 单独记录退出码。

## 归因与限制

- [官方 MoveGroupInterface 2.5.9](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp)
  的 `stop()` 发布 `trajectory_execution_event` 的 `stop` 消息，而非取消
  ExecuteTrajectory 的 UUID。本次复现了这个公开接口所用的停止路径。
- [官方 ExecuteTrajectory capability 2.5.9](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp)
  的互斥回调组执行阻塞等待，取消回调只接受请求，未调用 stopExecution。
  直接 Action 取消失效有原版运行证据，定位在 capability，不是证明 FJT
  controller plugin 整体有 BUG。
- [官方 ActionBasedControllerHandle 2.5.9](https://github.com/moveit/moveit2/blob/2.5.9/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/action_based_controller_handle.h)
  的阻塞取消在本次标准停止路径中正常返回。没有证明它必然死锁。
- 项目 `TerminalExecuteCapability` 反复调用 `waitForExecution(1ms)`，
  `compat` 的 retained future 修改部分用于适配这条自定义轮询路径；当前
  不能把这种适配一概称作官方确认的 BUG 修复。
- 未找到与整个 `compat` 修改集合逐项对应的官方确认 issue/PR。源码分析与
  本地复现不是官方维护者确认；没有证据证明所有修改都必须保留。
- 官方停止事件针对该 MoveGroup 的整个执行管理器，不提供本项目要求的
  外部目标隔离；父 PREEMPTED 也不能单独证明所有子控制器已停止。切换现有
  API 时必须在应用层解决执行所有权、迟到终态和实际状态确认，不能直接把
  stop event 当成当前 UUID 取消的等价替换。

这轮只新增独立核查脚本和报告。现有部署、任务接口、`compat`、自定义
capability 和运行配置未更换；尚未重新执行整机物理任务回归。
