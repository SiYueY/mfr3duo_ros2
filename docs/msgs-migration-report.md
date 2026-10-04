# mfr3duo_msgs 接口迁移报告

## 实现与来源

删除 mfr3duo_interfaces 及其旧构建/安装目录。新增独立 rosidl 包 mfr3duo_msgs，
仅包含 Move.action、Grasp.action、GraspEpsilon.msg，三份 IDL 原样取自 Franka 官方 Jazzy
commit `6cedf7f1a2ca280c433f643eae697be23eb2a15e`。
[来源及 SHA256](../mfr3duo_msgs/upstream.json)、许可证与 NOTICE 随包安装。
项目仍为 ROS 2 Humble，不依赖 franka_msgs，ROS 类型命名空间独立。

左右夹爪的 ~/move、~/grasp 与原 ~/gripper_cmd 共用 busy lease 和硬件 command/state。
Move 受阻不到位失败；Grasp 使用稳定闭合反馈与预期物体宽度内外容差，空夹/错误宽度失败。
取消保持实际开口，随后确认硬件更新与终止；停用和设备超时返回终止结果。
控制循环不分配或调用 DDS；Action 反馈与结果在非实时 monitor 中发送。
Control 增加 move_gripper、grasp_gripper、get_gripper_width；旧单指位置 API 保留。
初始化检查三个夹爪入口，Action 状态 SUCCEEDED 与设备 success=false 不会误判成功。

## 感知与任务验证

移除指定物体的双指接触 Topic、PointCloud2 和临时增加的接触点传输字段。
原 C++ 仿真接触查询保留。SensorAdapter 使用现有唯一 RobotHardware/Simulation 发布
/perception/objects/<id>/pose 与 /perception/tools/{left,right}/pose。
这些是明确标记的 simulation_world 仿真真值，不是相机检测；前缀可配置。
Robot 观察器按物体采样时刻精确匹配或在两侧各 50 ms 内插值工具位姿；不同 frame
使用该时刻非阻塞 TF 查询。缺失、过期、无效或不可匹配的样本返回 Unknown。

Pick 使用 Grasp 结果与实测总开口、真实抬升和相对稳定验证；既有物理阈值保持。
持物恢复要求当前开口、稳定相对位姿和此前确实验证过的抬升，静止接近不证明夹持。
Place 打开后验证支撑位置，再撤离工具并继续检查物体落位，最后恢复 world scene。
Unknown 不会自动打开夹爪或改写场景。新 Pick 与确认丢失后清除此前抬升证据。
历史 Phase 5/6 的服务及双指接触契约已被替代；本次不修改 MoveIt compat。

## 验证

- Humble 下八包重建通过。
- 固定上游 IDL 哈希、独立生成的 Python 字段/默认值检查通过。
- SDK 41/41、MuJoCo 后端 6/6、Hardware 11/11、Control 11/11 通过。
- 控制器软件测试覆盖左右新 Action、开口/速度换算、共享互斥、空夹与错误宽度、
  Move 受阻、非法参数、取消、停用和设备超时，并检查 update 零 new 分配。
- Control 测试覆盖新接口、SUCCEEDED/false 结果、取消 ACK 不等于终止、超时后资源保留、
  迟到结果恢复与旧入口兼容；500/1000 Hz 实际仿真控制回归通过。
- 观察器软件测试覆盖缺失、过期、无效位姿、时间错位、工具插值、缺失/有效 TF 和非法对象/手。
- 可视化整机 Navigate → Pick → Place 通过，任务耗时 49.3192 s，释放场景误差 0.000389 m。
- Robot 7/7 全部通过（总耗时 284.92 s），包含可视化整机演示、实际 Pick/Place、取消、
  Task deadline、未知观察进入 Error、重新初始化恢复、搬运中真实释放与 GraspLost/world scene 修复、
  活动任务 owner 析构、右夹爪实际空夹失败及再次打开、未验证抬升时 Unknown。
- 本轮相关测试合计 77/77（SDK 41 + 后端 6 + Hardware 11 + Control 11 + Robot 7 + Msgs 1）。
  colcon 全工作区缓存结果 64/64，其中 MoveIt 9、Nav 5 是本轮未重跑的历史结果，不计入本轮 77 项。
- ldd 确认 ROS 后端加载持久前缀 deps/romujoco-install 中的 SDK/MuJoCo，无缺失库。
- git diff --check 通过；docs/plan.md 原有暂存快照未改动，更新仅在工作树。

日志位于 /tmp/mfr3duo-msgs-*.log。首次硬件位姿测试因测试端读取启动期缓存而失败；
修正客户端等待新鲜匹配样本后 Hardware 全部通过，未放宽 0.3 s 年龄限制。

本次结果为源码、构建、软件测试及真实物理仿真验证，不是实机验证。
真实视觉生产者未实现；复用官方接口字段也不表示仿真夹爪拥有真实 Franka 固件的完整行为。
