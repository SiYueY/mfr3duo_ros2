# Phase 2B / Gate 2 Report

> 执行接口历史说明：本文记录当时的兼容插件/UUID 取消契约。当前已删除 compat 与自定义 capability，恢复原版官方执行入口，采用独占通道和应用层终止确认；见 [迁移报告](official-execution-report.md)。

Status: PASS. Gate 2: PASS (A, B, C). Gate D remains unsupported V1.1.

2026-10-04 follow-up: this report describes acceptance of the modified execution
path, not proof that every compatibility change fixes an upstream bug. The
[original-component audit](upstream-execution-audit.md) verifies that the
unmodified official controller plugin executes and stops through the official
stop-event path. Direct ExecuteTrajectory Action cancellation and all-child
terminal confirmation have different behavior; do not attribute the entire
compatibility subset to confirmed official controller-plugin defects.

## Implemented and modified files

`mfr3duo_moveit/config/gate2_manifest.yaml` fixes eight targets, ten IK seeds per
target, start state, collision scene, solver/planner versions, tolerances,
timeouts, success rate >= 0.9 and P95 planning wall time <= 5.5 seconds.
`tools/joint_planning_probe.cpp` samples seeded single-attempt KDL IK, merges both
poses and explicit spine into one AND constraint, validates the complete merged
state, plans exact groups and checks all trajectory points. Runtime uses measured
fresh states and the same TEM/JTC/hardware instance. `test/gate2_test.py` requires
PASS and clean process exit.

Humble 2.5.9's blocking accepted callback prevented timely ExecuteTrajectory
cancel processing. The private `src/terminal_execute_capability.cpp` now reserves
one execution owner, accepts nonblocking, cancels exact goals and polls every
selected controller's actual terminal result before releasing the resource.
`compat/` retains the BSD-licensed standard FJT controller manager subset with
retained per-goal result futures, bounded goal response, late-accept cancellation,
nonblocking cancel requests, interruptible bounded TEM waits and atomic status.
Both private plugins build in this package; the system MoveIt installation is
unchanged. No additional facade executor/thread or second TEM is created.
Builtin MoveAction is planning-only; execution uses ExecuteTrajectory. Changes
also include plugin XML, package/CMake dependencies, launch and tests.

## Build and tests

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/siyuey/workspace/mfr3duo/deps/romujoco-install:$CMAKE_PREFIX_PATH
colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-phase2b-final-test-logs
colcon test --parallel-workers 1 --event-handlers console_cohesion+
colcon test-result --verbose
```

All seven workspace packages built; **39 tests, 0 errors, 0 failures, 0 skipped**.
Logs: `/tmp/mfr3duo-phase2b-final-{build,tests,results}.log`.

Reproduce only Gate 2 with `python3 mfr3duo_moveit/test/gate2_test.py acceptance`;
cancellation software fixture: same command with `cancel`.

## Acceptance and runtime validation

- [PASS] Eight targets x ten trials = 80/80 plans. Per-case P95: 0.0713–0.1451 s.
- [PASS] Gate A high target fixed-arm distances 1.77966/1.78474 m exceed the
  conservative chain translation-norm upper bound 1.42266 m by 0.357/0.36208 m.
  Sampled spine participates; real high executions move spine 0.611416/0.648122 m.
- [PASS] Gate B merges both IK solutions and checks both constraints, bounds,
  self/world collision. Actual spine drift 0.000618277 m < 0.003 m.
- [PASS] Gate C uses the same explicit spine value for both arm IK samples;
  actual C_up spine displacement 0.0992074 m reaches the explicit target.
- [PASS] Four actual runtime cases, home resets and full collision/inactive checks.
  Maximum TCP position error 0.004556 m; orientation error 0.007742 rad.
- [PASS] Negative fixtures detect a one-sided constraint violation, a TCP world
  obstacle and IK sampling failure on a distant target. Finite search failure is
  reported as IKFailed, not mathematical NoIKSolution.
- [PASS] Actual MuJoCo/TEM cancellation confirms CANCELED before home recovery.
- [PASS] Software delayed-terminal fixture proves ACK != terminal, first child
  terminal is insufficient, unresolved children block new execution, late
  terminals converge and the next execution succeeds.
- [PASS] Phase 2A single-group runtime and all prior regressions remain passing.

## Known limitations / unresolved issues

The fixed acceptance scene is empty except for the negative obstacle fixture.
This proves the specified targets and trials, not arbitrary poses or environments.
Camera mount placement remains provisional; no Octomap updater is configured.
No public MoveGroup facade or Cartesian capability is claimed yet. Free shared
spine dual-arm Gate D is not implemented. No Phase 2B blocker remains.
