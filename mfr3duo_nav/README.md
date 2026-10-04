# MFR3Duo 全向导航

`Navigator` 通过 Nav2 的标准 NavigateToPose / NavigateThroughPoses action 执行导航。
应用传入并运行 `rclcpp::Node`；库不创建线程或 Executor，不链接 Control、MoveIt 或 RobotHardware。
安装后的 CMake target 为 `mfr3duo_nav::mfr3duo_nav`。

`initialize(timeout)` 在一个总截止时间内检查地图、有效且新鲜的 odom、map→base_link TF、
八个 Nav2 lifecycle 节点 ACTIVE 及 action servers。`config/facade.yaml` 定义状态年龄、
goal response、导航与终止确认截止时间。同步 `navigate_to/navigate_through` 等待真实终止；
`start_navigate_to` 返回可复制 Handle。Busy 或输入错误也保存在返回 Handle 中。
取消只操作本 facade 的 UUID；取消 ACK 不代表终止。终止未知时禁止新导航，迟到真实结果可使状态收敛。

```cpp
#include <mfr3duo_nav/navigator.hpp>
// 应用负责在其他线程运行 node 的 Executor。
mfr3duo_nav::Navigator navigator(node);
auto ready = navigator.initialize(std::chrono::seconds(30));
if (ready) {
    auto handle = navigator.start_navigate_to(map_goal);
    auto result = handle.wait();
}
```

导航使用 `mjcf/navigation.xml` 和匹配的静态地图，AMCL 使用 OmniMotionModel 与前 LiDAR，
两个 costmap 使用前后 LiDAR。DWB 允许双向 x/y 运动，Twirling 评分抑制绕障途中不必要的旋转；
RotateToGoal 保留目标姿态对齐。最终平滑输出 remap 到 `/tmr_controller/cmd_vel`。
全栈使用 ROS wall time；唯一 odom→base_link TF 和 `/tmr_controller/odom` 由 TMR controller 发布。

**移动前必须使用 MoveGroup 到达 `config/navigation_posture.yaml` 的固定双臂及 spine 姿态。**
当前 footprint 包括该姿态的上身与手臂，不能用于任意伸臂或携物时的严格碰撞保证。
Nav launch 本身只启动控制及 Nav2；不会替应用执行姿态移动。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mfr3duo_nav nav.launch.py viewer_enabled:=true
# 已有 control 时使用 start_control:=false，避免创建第二个硬件实例。
```

完整真实 POC 自动启动一个共享硬件实例及 MoveIt，先执行固定姿态，然后验证前进、横移、
对角、原地转向、多点导航、物理绕障、定位连续性、精确取消和实测停稳：

```bash
export ROS_LOG_DIR=/tmp/mfr3duo-nav-test-logs
python3 mfr3duo_nav/test/runtime_test.py
```

该 POC 需要从工作区根运行，并有本机 DDS/图形渲染访问能力；默认关闭 viewer。
纯接口、延迟 action 和安装包检查由 CTest 执行。TMR encoder odometry 的正常/慢速仿真一致性测试
属于 `mfr3duo_control`。同实例 world pose 仅用于验收，不输入 AMCL 或 odom 估计。
