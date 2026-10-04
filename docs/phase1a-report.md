# Phase 1A 实施报告

结论：PASS，允许进入 Phase 1B。此次不启动 MoveIt/Nav2，不提前实现上层 execution。

## 实现

- 新增 mfr3duo_control：安装 controller plugin、config/launch 和纯 C++17 TmrKinematics。
- 双臂保留 JTC，spine 使用单关节 JTC；控制率与 hardware control_period 共用 launch 参数。
- TMR body twist 实现实际有符号几何、最近转向/反向轮速、统一饱和、转向与加速度限制、单调时钟 watchdog、零速/停用 hold 和重新激活隔离。
- TMR 内部转向连续，输出编码为 [-pi, pi]，与现有后端 position 限幅及最短角误差兼容。
- hardware 保留一条 RobotHardware 路径；传感器与 broadcaster 的底层回归独立于 Control 配置。
- 同步更新运行入口和架构说明；docs/plan.md 的用户暂存内容未修改。

## 相邻项目修复

robot_mujoco/romujoco 的 consume_camera_results 首次完成可能晚于控制启动；缓存的未发布 RobotState 和聚合状态还会持有嵌套快照，触发不必要的池增长。

初始化准备固定相机集合的发布/写入存储及 vector capacity；更新前释放仅由池拥有的缓存快照中的嵌套引用。读者持有的快照不修改，实际保留读者仍允许池扩容。新增确定性回收/不可变性测试，原 allocation assertion 不放宽，相机不关闭。

## 验证

- ROS 2 Humble 全部 7 包构建成功。
- 整仓 colcon test：31 tests，0 errors，0 failures，0 skipped。
- romujoco Release 构建及全部 38 tests 成功，包括 snapshot、并发、传感器及安装包 consumer。
- 实际 MuJoCo 500/1000 Hz：双臂/spine FJT 终止 SUCCESS、2 秒轨迹时长、位置误差、关节及速度限位；无效 joint goal 拒绝。
- TMR 实测前进、横移、旋转、反向驱动、零速 hold、watchdog 停轮、停用/重激活成功；单元测试覆盖 pi/2、pi、跨 pi 连续转向与 invalid command 不刷新 watchdog。
- 500/1000 Hz 控制线程零 C++ new 基准各连续运行 20 次，全部通过，同时运行整仓回归制造负载。
- Control 和 MuJoCo 的 out-of-tree installed-package consumer 构建/运行成功。
- git diff --check 成功。

日志位于 /tmp/mfr3duo-phase1-camera-prepared-tests.log、/tmp/mfr3duo-phase1-camera-prepared-results.log、/tmp/mfr3duo-phase1-allocation-prepared.log 和 /tmp/mfr3duo-phase1-backend-tests.log。

## 环境和边界

后端安装前缀为 /home/siyuey/workspace/mfr3duo/deps/romujoco-install，ROS_LOG_DIR 使用可写 /tmp 目录。已有 /tmp/mfr3duo-romujoco-install 不再存在；此次已实际重新构建及安装后端。

控制周期参数表示目标周期，不保证操作系统墙钟硬实时；测试验证测量段的线程分配及运行正确性。夹爪 action、finger projection、Control facade 属于 Phase 1B，odom 属于 Phase 4。

## 复测

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-ros-logs
colcon test --executor sequential --event-handlers console_cohesion+
colcon test-result --verbose
ctest --test-dir build/mfr3duo_hardware -R control_cycle_benchmark --repeat until-fail:20 --output-on-failure
```
