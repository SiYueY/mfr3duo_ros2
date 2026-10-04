# Phase 3 Report

> 执行接口历史说明：本文记录当时的兼容插件/UUID 取消契约。当前已删除 compat 与自定义 capability，恢复原版官方执行入口，采用独占通道和应用层终止确认；见 [迁移报告](official-execution-report.md)。

Status: PASS. Gates 1 and 2 remain PASS. Phase 4 is next.

## Implementation

`mfr3duo_moveit/include/mfr3duo_moveit/{move_group.hpp,moveit_types.hpp}` and
`src/move_group.cpp` implement the public PImpl facade and read-only value Plan.
The application spins the supplied node; the facade creates no executor or thread
and links neither Control nor RobotHardware. Installed CMake consumers link
`mfr3duo_moveit::mfr3duo_moveit`, with the complete ROS/MoveIt/OMPL dependencies exported.

Builder mutations are atomic and preserve explicit group/target ownership.
Joint, pose, position-only and orientation-only targets use the seven exact SRDF
groups. Both arm goals and explicit spine constraints form one AND goal.
Free-spine single-arm sampling uses the actual eight-axis KDL chain; dual-arm
sampling uses one shared fixed spine value. Gate D remains unsupported. Full
bounds, self/world collision and merged constraints are checked before planning,
at trajectory waypoints and during execution. Fresh scene attachments remain in
the state used for collision validation; stored Plan state only freezes joints.

Initialization checks complete fresh measured joints, active arm/spine
controllers, model groups, planning scene and execution service availability.
A plain node can retrieve the running MoveIt node's model parameters. TF input is
collected through weak callbacks and resolved at one measured snapshot stamp.
Humble callback-group guard lifetime handling follows the Control facade; model
and facade state are not retained after destruction.

Execute validates the Plan's own groups, joint sets, vectors, finite values,
timestamps, limits and start positions. Builder changes cannot reinterpret a
stored Plan. Only the owned ExecuteTrajectory UUID is canceled. A cancellation
ACK does not release execution ownership; unresolved termination blocks the next
operation until a late terminal result arrives. Pending late acceptance remains
cancelable after facade destruction, with client ownership released at terminal
or context shutdown.

Cartesian interpolation is single-arm with fixed spine and other arm, common
PoseStamped frame, full bounds/collision/path-constraint/jump checks and TOTG
timing. Partial paths retain a private fraction in Plan and remain non-executable
even if the caller changes the public diagnostic fraction. Validation constants
are in `config/facade.yaml`.

MoveIt 2.5.9's OMPL planner installs a global raw logger pointer owned by each
planner instance. Destroying a second facade caused the first to crash in
`ompl::msg::log`. Facade initialization now restores a shared ROS logger with a
lifetime exceeding all facade pipelines, including construction failure paths.
The real runtime test retains the two-instance destruction regression.

## Tests and commands

Tests: `builder_test.cpp`, `installed_consumer_test.py`, `facade_action_probe.cpp`,
`facade_runtime_probe.cpp`, driven by `facade_runtime_test.py` and CTest. Runtime
requires both its PASS marker and a clean process exit. The delayed-action fixture
covers rejection/failure, response and execution deadlines, late accept/reject,
cancel ACK without terminal, foreign goal isolation, late terminal convergence
and destruction during pending response. Actual runtime covers all required
planning combinations, exact groups, invalid Plan mutations, old start state,
Cartesian full/partial/jump/frame/collision cases, actual execution and cancel,
busy mutations and repeated facade destruction. A newly attached object that
collides only at the stored trajectory's endpoint verifies fresh scene handling.

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/siyuey/workspace/mfr3duo/deps/romujoco-install:$CMAKE_PREFIX_PATH
colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-phase3-final-test-logs
colcon test --parallel-workers 1 --event-handlers console_cohesion+
colcon test-result --verbose
```

The final build passed all seven workspace packages. The final regression passed
**43 tests, 0 errors, 0 failures, 0 skipped**, including all four facade tests,
Gate 2 acceptance, delayed-child termination and 500/1000 Hz Control runtime.
Build logs: `/tmp/mfr3duo-phase3-guard-final-build.log` and
`/tmp/mfr3duo-phase3-result-build.log`. Final test/results logs:
`/tmp/mfr3duo-phase3-complete-{tests,results}.log`.

Full-state bounds validation exposed a measured spine position of
0.850146 m during high-pose return, exceeding its unchanged 0.85 m limit. The
MuJoCo spine position controller damping has been raised from 1000 to 1800 N s/m;
joint range, goal/path limits, manifest targets and acceptance tolerances remain
unchanged. The diagnostic rerun reports the violating variable and limits. Logs are
`/tmp/mfr3duo-phase3-final-{build,tests,results}.log` and
`/tmp/mfr3duo-phase3-bounds-tests.log`. A 2000 N s/m trial passed Gate 2 once but
failed the existing large-step settling test; 1800 passes its original transient
and settling criteria. Joint limit constraints now activate 0.001 m/rad before
the mechanical stop; the physical ranges are unchanged. Further testing also
exposed a saturated right-arm IK solution crossing joint5's limit. Both samplers
reject redundant IK solutions that fail to leave the existing 0.02 rad / 0.003 m
execution-error envelope inside the joint range. Explicit joint targets are not
modified, measured states are not clipped, and actual bounds checks remain strict.
The fixed Gate 2 manifest, target constructions, seeds and thresholds are unchanged.
Final rebuilt workspace validation passed with these changes.

The independent sibling `mfr3duo_mujoco` still referenced old TMR servo actuator
names while its model provides force motors. Its C++ configuration now supplies
the actual names and explicit 30/5/2 steering stiffness/damping/drive damping.
RoMuJoCo's private dynamic swerve implementation supports those explicit gains,
feedback torque, gear conversion and control/force limits, retaining its original
native-servo path. Its test covers both paths and rejects motors without gains.
Backend 39 tests passed, and sibling four tests passed with both a fresh private
install and the production workspace prefix. Sibling
camera tests require local graphics access; the default sandbox run timed out
creating its initial EGL frame, while the local elevated run passed. Logs:
`/tmp/mfr3duo-phase3-swerve-{build,tests,production-install}.log` and
`/tmp/mfr3duo-phase3-sibling-{production-build,production-tests}.log`.

## Acceptance

- [PASS] Atomic builder, explicit ownership, no target, exact support matrix and
  Gate D rejection; no public mutable Plan references.
- [PASS] Actual free-spine left/right high-goal planning, fixed-spine dual pose
  AND goal with explicit spine, position-only and orientation-only targets.
- [PASS] Stored Plan retains its groups; stale start and malformed plans reject.
- [PASS] Full/partial Cartesian, common frame, jump, path constraints, collision,
  time parameterization and private fraction execution checks.
- [PASS] Freshly attached object affecting a stored path is collision-checked.
- [PASS] Real arm/spine execution, owned cancellation, busy mutation rejection
  and repeated destruction while the application executor remains running.
- [PASS] Delayed response/result state machine, foreign goal isolation and
  owner destruction with pending response. Remote execution timeout is reported
  as ExecutionFailed with the original MoveIt code; local deadline remains Timeout
  only after termination confirmation.
- [PASS] Installed independent CMake consumer and complete prior regressions.
- [PASS] Gate 2 fixed manifest again reaches 80/80 successful plans, actual
  A/B/C execution and terminal cancellation without loosening criteria.

## Limits and next Gate

No Nav facade, odometry, task orchestration or physical grasp capability is claimed
here. The current world scene is empty except for temporary collision fixtures.
Physical scene/observation synchronization belongs to Phase 5. Phase 4 starts
now that Phase 3 acceptance and all required regressions have passed.
