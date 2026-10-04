# Phase 6 Report

> 接口迁移说明：下文的自定义 ObserveGrasp 服务和双指接触契约为历史实现。
> 当前使用 mfr3duo_msgs 夹爪动作及标准物体/工具位姿 Topic，见 [迁移报告](msgs-migration-report.md)。


Status: PASS. Phase 6 and final V1 acceptance passed. Gates 1–3 remain PASS;
Gate D/free shared-spine remains unsupported V1.1. Validation: 2026-10-04.

## Delivered implementation

The public C++17 library exports Robot, RobotTask, NavigateTask, PickTask,
PlaceTask, TaskSequence, TaskHandle, TaskResult and their value types in separate
headers. Robot uses PImpl and is neither copyable nor movable. TaskHandle retains
its shared terminal record after Robot destruction. Synchronous execute deep
clones the entire input; asynchronous start transfers unique ownership. Nested
sequences are serial and stop on the first failed, canceled or unknown operation.
Robot admits one top-level task. Invalid input, concurrent admission and missing
readiness fail before motion. Absolute deadlines include child deadlines and
check chrono overflow.

The application supplies the node and executor. Robot has no private executor or
thread. Its bounded worker, deadline monitor and observation health callbacks use
external callback groups. Control owns gripper execution, MoveGroup owns planned
and full-fraction Cartesian arm motion, Navigator owns navigation, and the scene
and private observer confirm physical state. Cancellation targets the current
owned SDK operation. Acknowledgment does not mean termination. Unconfirmed
termination leaves Robot in Error and prevents a new task; late results do not
replace the original TaskHandle result. Explicit initialize waits for actual
termination and restores scene state from physical observations.

Initialize checks Control, MoveGroup, scene service, Nav2 lifecycle/map/localized
TF and same-instance GraspObserver against one total deadline. Shared bringup
starts one hardware/controller manager; MoveIt and Nav2 do not start additional
hardware. Simulation, controllers, transforms and observations use the same
robot model and wall-time policy.

Pick opens the selected gripper, plans a pregrasp with explicit spine height,
executes a complete Cartesian approach, and verifies blocked width, sustained
bilateral contacts and physical stability before attachment and a 10 cm lift.
Success requires actual lift and at most 5 mm/0.05 rad relative drift over the
frozen observation window. Place checks the actual held object, plans approach,
opens, verifies released/resting physical pose, detaches into the measured world
pose, restores collision permissions and retreats. Failures preserve a confirmed
held attachment, update a confirmed dropped object's world pose, or enter Error
without inventing a holding state. Navigation with a held object is outside V1.

The support table and target object are refreshed together from a fresh physical
observation before Pick, and the table is refreshed before Place. MoveIt's model
frame is base_link; a table configured before navigation would otherwise remain
at an obsolete base-relative pose. The whole-task test checks the released object
relative to the support table in the final PlanningScene, including origin and
primitive transforms. The same table/object refresh is repeated immediately before
attachment, after the arm has settled.

## Main file locations

- Public API: mfr3duo_robot/include/mfr3duo_robot/{types,task_result,task,
  navigate_task,pick_task,place_task,task_sequence,task_handle,robot}.hpp.
- Ownership/value implementation: mfr3duo_robot/src/task.cpp; orchestration,
  readiness, deadlines and recovery: mfr3duo_robot/src/robot.cpp.
- Shared bringup/demo: mfr3duo_robot/launch/robot.launch.py,
  config/robot.yaml, maps/task_map.*, tools/task_demo.cpp and package/CMake exports.
- Permanent validation: test/task_input_test.cpp, robot_runtime_probe.cpp,
  robot_runtime_test.py, robot_demo_test.py and robot_launch_contract_test.py.
- Gravity fixes: both mfr3duo_mujoco/src/config.cpp checkouts and their
  test/arm_hold_response_test.cpp/CMakeLists.txt; SDK joint_component.cpp and
  joint_gravity_compensation_test.cpp.
- Gripper dwell: mfr3duo_control controller implementation and unit regression;
  auxiliary home reset: mfr3duo_moveit/tools/joint_planning_probe.cpp;
  collision diagnostics: mfr3duo_moveit/src/move_group.cpp; synchronized truth
  comparison: mfr3duo_nav/test/navigator_runtime_probe.cpp.
- Documentation: top-level README.md, docs/architecture.md, this report and
  mfr3duo_robot/README.md. docs/plan.md is not modified.

## Physical and regression fixes

Both MuJoCo backend checkouts now enable arm position-controller gravity
compensation. Without it, reissuing the measured position as a Cartesian start
replaced the old desired hold and introduced gravitational sag; the object could
shift over 5 mm relative to the hand during lift despite bilateral contact. Joint
gains, effort caps and acceptance thresholds are unchanged. The SDK calculates
pure gravity in its preallocated scratch data using kinematics, center-of-mass
and recursive Newton–Euler stages, without repeated full reset/collision/solver
work. Independent full-forward reference checks cover moving two-joint states and
confirm that live velocity is preserved. Both backend regression tests verify
four measured-position handoffs on both arms across the entire transient.

Gripper blocking confirmation also waits for measured contact effort to settle;
an increasing preload cannot finish the blocking dwell early. The controller
retains its effort cap and requires sustained low velocity and a blocked target.

The measured resting box has about 21 micrometres of compliant penetration into
the physical table. Exact rigid planning surfaces would reject the lift start as
box/grasp_table collision. The planning table has a fixed 0.1 mm surface skin
(height .9998 m instead of the physical 1 m, with the measured center unchanged).
The whole-task test freezes that representation explicitly. Physical table/box
geometry, measured poses, all physical success thresholds, other objects and the
AllowedCollisionMatrix remain unchanged; object/table collision is not disabled.
Invalid-start diagnostics now include up to four actual collision pairs/depths.

The Gate 2 auxiliary reset recognizes an already bounded, collision-free home
state within 1 mrad/1 mm. Such a zero-displacement reset need not execute a
one-point retimed trajectory. Every actual acceptance trajectory retains the
existing strict timed-path validation, fixed targets, seeds and tolerances.
Navigation truth comparison uses localized transforms at the physical sample's
time, with a bounded wait for that transform. The sample mutex is released before waiting so the TF and observation callbacks
can continue on the external executor. Truth does not feed AMCL. Position
and heading error limits remain 0.1 m and 0.15 rad.

## Verification

Fresh full eight-package build passed after all corrections (31.3 s). Fresh
affected-package builds also passed for the scene and diagnostic corrections. The final affected-package regression
passed MoveIt 9/9, Navigator 5/5 and Robot 6/6. The five other packages retain
their passing full-regression results after the gravity/gripper fixes; their
implementation did not change during the final scene/test corrections.

`colcon test-result --all --verbose` reports 62 test records, zero errors, failures
or skips: 49 CTests plus 13 description pytest detail records (the description
CTest also wraps those pytest cases). The interfaces-only package has no runtime
tests. Runtime assertions inside probes include substantially more scenarios.

Verified with the gravity and gripper fixes:

- SDK: 41/41 CTests; independent backend: 5/5 CTests.
- Main backend: 6/6; hardware: 11/11; Control: 11/11.
- Hardware control loops at 500/1000 Hz: zero C++ allocations in the control
  thread and zero overruns. Pure hardware 1000 Hz mean 270.3 us, P99 543.3 us,
  maximum 784.3 us; adapter 1000 Hz mean 256.6 us, P99 332.8 us, maximum 505.8 us.
- Navigator: final 5/5 passed, including detour and physical footprint checks.
  Maximum same-time localized/truth position error 0.0733111 m and heading
  error 0.0315326 rad (unchanged limits 0.1 m/0.15 rad).
- MoveIt: final 9/9 passed; all 80 fixed target/seed cases and runtime Gate 2 passed.
- Corrected public whole Navigate → Pick → Place passed in actual simulation.
  Physical lift 0.099588 m, maximum relative drift 0.000261 m, angle 0.000607 rad.
  Final scene support-relative placement error 0.002925 m (< 0.01 m); fixed
  planning support dimensions and confirmed world detachment passed.
- Public Task runtime: Pending cancel/deadline, RobotBusy, real physical Pick and
  Place, retained results, nested clone isolation, canceled-held recovery,
  observer loss while idle/active, explicit recovery, active timeout, unknown
  termination followed by late terminal recovery, actual external gripper release
  producing GraspLost and confirmed world detachment, sequence short circuit,
  and bounded owner destruction during actual motion all passed.
- Observed public Pick lift: 0.099521 m; maximum relative drift 0.000312 m and
  angle 0.001313 rad, with the original 0.005 m/0.05 rad limits.
- Installed consumer builds using only find_package(mfr3duo_robot) and the
  exported mfr3duo_robot::mfr3duo_robot target, and runs successfully against
  the final installed libraries.

The initial full regression exposed the zero-motion home-reset assertion and
asynchronous navigation truth comparison. Refreshing the support scene then
exposed the compliant-contact lift-start collision. Corrected full affected-package
reruns passed; failed runs are not counted as passing evidence.

Session logs are in /tmp/mfr3duo-phase6-completed-tests.log and
/tmp/mfr3duo-phase6-completed-results.log. Final full build and installed consumer
logs are /tmp/mfr3duo-phase6-completed-build.log and
/tmp/mfr3duo-phase6-completed-consumer.log. Unchanged package full-regression
results are in /tmp/mfr3duo-phase6-final-tests.log; SDK and independent backend
results are in /tmp/mfr3duo-phase6-sdk-fast-tests.log and
/tmp/mfr3duo-phase6-root-tests.log. These session logs are ephemeral; the permanent
tests and commands below reproduce their checks.

## Repeatable commands

Run from mfr3duo_ros2:

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/siyuey/workspace/mfr3duo/deps/romujoco-install:$CMAKE_PREFIX_PATH
colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-robot-logs
colcon test --executor sequential --event-handlers console_cohesion+ --return-code-on-test-failure
colcon test-result --all --verbose
ros2 launch mfr3duo_robot robot.launch.py run_demo:=true
```

The viewer is enabled by default. run_demo defaults to false for an application
that owns its own Robot. The demo uses the task_home initial keyframe, calibrated
static map/simulation_world transform, task map and 50 g box fixture. No runtime
teleport, welding or clipping is used. The application example owns four executor
workers and keeps them running until Robot has been destroyed.

SDK and independent backend checks, from the workspace root (stop running
simulations before rebuilding/installing shared libraries):

```bash
cmake -S robot_mujoco/romujoco -B robot_mujoco/romujoco/build \
  -DCMAKE_BUILD_TYPE=Release -DROMUJOCO_BUILD_TESTS=ON \
  -DCMAKE_INSTALL_PREFIX="$PWD/deps/romujoco-install"
cmake --build robot_mujoco/romujoco/build -j2
ctest --test-dir robot_mujoco/romujoco/build --output-on-failure
cmake --install robot_mujoco/romujoco/build
cmake -S mfr3duo_mujoco -B /tmp/mfr3duo-phase6-native \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DMFR3DUO_MUJOCO_BUILD_TELEOP=OFF \
  -DCMAKE_PREFIX_PATH="$PWD/deps/romujoco-install" \
  -DMFR3DUO_DESCRIPTION_SHARE_DIR="$PWD/mfr3duo_description"
cmake --build /tmp/mfr3duo-phase6-native -j2
ctest --test-dir /tmp/mfr3duo-phase6-native --output-on-failure
```

## Validation boundaries

Physical whole-task evidence concerns the configured left/Auto hand and this
fixture at the default 500 Hz. It does not establish arbitrary object grasping,
right-hand physical whole tasks, carried-object navigation, real hardware,
perception, Agent/frontend integration or V1.1 free shared-spine planning.
Right-hand observation and lower-level planning are covered separately by prior
reports. Navigation's earlier slower-physics 1000 Hz probe covered forward motion,
not the complete whole-task scenario. No physical acceptance threshold was
relaxed. docs/plan.md remains untouched, including its user's staged contents.

## Desktop viewer and startup interruption follow-up

The user's /tmp/mfr3duo-robot-logs run received Ctrl+C about two seconds after
launch, while hardware was still configuring. The reported invalid-context,
MoveIt/Nav2 abort and hardware activation errors occurred after SIGINT. No
pre-interrupt task failure was recorded. Early INFO messages waiting for odom/map
transforms are readiness waits.

Control initialization now checks ROS context validity in its bounded wait loop.
The demo retains callback-group shells until its external executor has joined and
skips explicit node removal after context shutdown. This avoids Humble's invalid
callback-group cleanup path without delaying context shutdown until a blocked
SDK operation completes. Initialization SIGINT is reported as
ROBOT_DEMO_INTERRUPTED with exit status 130.

Permanent robot_demo_initialization_shutdown runs the actual installed demo with
absent hardware, sends SIGINT, and requires exit within three seconds without
abort or forced termination. Control's existing action regression also passed.
The final interruption test passed three consecutive runs, exiting in
0.120–0.123 s and reporting ROBOT_DEMO_INTERRUPTED. The desktop viewer was
confirmed as the actual MuJoCo : mfr3duo_manipulation window. The final viewer
run passed the complete Navigate/Pick/Place task and clean demo process exit:
lift 0.099575 m, relative drift 0.000273 m, orientation drift 0.000545 rad, and
final support-relative placement error 0.003522 m. Physical limits are unchanged.
Logs are /tmp/mfr3duo-viewer-followup-shutdown-test.log,
/tmp/mfr3duo-viewer-followup-control-test.log and
/tmp/mfr3duo-viewer-followup-demo.log. These
checks supplement the earlier full regression; they do not claim a new full
all-package test run. Upstream MoveIt/Nav2 startup-interruption diagnostics are
not represented as successful initialization.

Reproduce the viewer test from mfr3duo_ros2 after sourcing install/setup.bash:

```bash
ctest --test-dir build/mfr3duo_robot -R '^robot_demo_initialization_shutdown$' -V
MFR3DUO_TEST_VIEWER=true python3 mfr3duo_robot/test/robot_demo_test.py
```

The desktop viewer check uses the same physical whole-task assertions and final
PlanningScene verification as the headless test. Viewer-disabled runtime tests
alone do not establish desktop-viewer behavior.

## Demo speed follow-up

The earlier viewer run spent approximately 50 s in the 0.5 m pregrasp spine
transfer, using the preceding navigation posture's 0.1 velocity factor. Precise
Cartesian stages used 0.05/0.02 velocity/acceleration factors.

Demo robot.yaml now overrides empty-hand transit factors to 0.30/0.20 and
carrying/Cartesian factors to 0.10/0.05. The Robot facade explicitly sets transit
factors for navigation posture/pregrasp and precision factors before carrying
preplace. A new pregrasp therefore cannot inherit the preceding Cartesian
factors. Without demo overrides, transit defaults remain 0.1/0.1 and precision
factors use the frozen grasp.yaml values. Contact confirmation, physical drift
limits, lift distance, observation windows and joint limits are unchanged.
The demo logs task_seconds after readiness.

The faster desktop-viewer run passed: task_seconds=55.8088, versus approximately
110 s from the earlier run's first transport plan to completion. Actual lift
0.099690 m, relative drift 0.000158 m, orientation drift 0.000161 rad, and final
support-relative placement error 0.003547 m all satisfy the existing thresholds.
The full Robot regression passed 7/7, including the new initialization shutdown
test and all actual cancellation/deadline/unknown termination/drop/recovery/owner
destruction cases. The headless full-task run passed in 56.4884 s, with physical
lift 0.099686 m, drift 0.000161 m, angle 0.000050 rad and placement error
0.003285 m. The all-package result aggregate reports 63 records and zero errors,
failures or skips; unaffected packages retain their earlier passing results.
Viewer results are in /tmp/mfr3duo-speed-viewer.log; build results are in
/tmp/mfr3duo-speed-build.log, Robot regression in /tmp/mfr3duo-speed-tests.log,
and aggregate results in /tmp/mfr3duo-speed-results.log.

## Further demo speed increase

Only demo robot.yaml changes in this follow-up: empty-hand transit factors are
0.50/0.30, and precision/carrying factors are 0.15/0.08. Facade defaults, frozen
grasp.yaml, controller gains, limits and physical success checks are unchanged.
The source/install config symlink was verified before running both real demos.

Desktop viewer: PASS, task_seconds=45.7045 (previously 55.8088). Lift 0.099741 m,
relative drift 0.000131 m, orientation drift 0.000060 rad, final support-relative
placement error 0.000550 m. The demo process exited normally.

Headless whole-demo CTest: PASS, task_seconds=49.1694. Lift 0.099724 m, relative
drift 0.000135 m, angle 0.000078 rad and placement error 0.000403 m. Timing varies
with planning and localization; these are measured runs, not a fixed performance
guarantee. Durations exclude readiness.

Logs: /tmp/mfr3duo-speed2-viewer.log and /tmp/mfr3duo-speed2-headless.log; the latter
is the CTest result, with detailed successful headless output in
build/mfr3duo_robot/Testing/Temporary/LastTest.log. Only the affected whole-demo
checks were rerun for this configuration-only change; the prior 7/7 Robot
regression is retained as earlier evidence, not claimed as a new full run.
