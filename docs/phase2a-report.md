# Phase 2A Report

Status: PASS. Gate status: N/A; Gate 1 was the prerequisite and remains PASS.
Gate 2 joint pose-goal capabilities are not claimed by this phase.

## Implemented

Seven exact SRDF planning groups (7/7/1/8/8/14/15 active variables), separate
gripper end-effectors, left/right TCP chains, KDL for arm and real arm+spine
chains, URDF-bounded joint velocities and conservative acceleration limits, OMPL
RRTConnect, time parameterization and three FollowJointTrajectory controller
mappings. MoveIt launch optionally includes Control once, with wall time and the
single controller update-rate parameter. No Control C++ library is linked.

The POC checks model contracts and an environment-collision negative fixture,
complete fresh measured state, actual TCP/spine TF, joint limits, trajectory time
and velocity, full-state collision at every trajectory point, exact participating
joint sets, real TEM execution, final error and inactive arm/spine error <0.003.
The test requires the POC to exit cleanly after its PASS marker.

## Modified files

- `mfr3duo_moveit/{CMakeLists.txt,package.xml,README.md}`, `config/` (SRDF,
  kinematics, joint limits, OMPL, controllers), `launch/moveit.launch.py`,
  `tools/planning_probe.cpp`, `test/{config_contract_test.py,runtime_integration.py}`.
- Control: broadcaster configuration restricts real joints to position/velocity;
  `src/control.cpp` ignores packets containing only passive joints, and its action
  test continues publishing passive packets during the controlled-state stale test.
- Hardware: public sensor types/API and implementation forward the existing
  MuJoCo passive-state API; SensorAdapter publishes five unactuated chassis joints
  outside the control loop. Hardware snapshot tests require matching timestamps
  and finite passive samples. The 21/74/53 ros2_control contract is unchanged.
- Description submodule: sensor/mount/rocker collision geometry follows existing
  visual geometry; static URDF regenerated and regression added.
- Root README and architecture reflect the implemented boundary.

SRDF excludes adjacent/fixed rigid assemblies and eight explicitly listed
mechanical self-contact pairs (six wheel/housing, two closed finger pairs).
Other self and world collision pairs stay enabled. A box at the TCP is rejected.
No broad collision disable or joint-limit relaxation was used.

## Build

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/siyuey/workspace/mfr3duo/deps/romujoco-install:$CMAKE_PREFIX_PATH
colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-phase2a-final-tests-ros-logs
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

All seven packages rebuilt successfully. Final workspace result: **37 tests,
0 errors, 0 failures, 0 skipped**. Includes description, MuJoCo snapshots/consumer,
hardware sensors/concurrency/allocation/lifecycle, Control action/termination,
500/1000 Hz real runtime and installed consumer, plus MoveIt contract/runtime.
Logs: `/tmp/mfr3duo-phase2a-final-{build,tests,results}.log`.

## Runtime validation / acceptance

- [PASS] KDL loads for all four single-tip chains; arm+spine contains actual spine DOF.
- [PASS] Seven exact group dimensions, no finger/TMR joints in planning groups.
- [PASS] Entire measured state including passive joints is complete and fresh.
- [PASS] LeftArm, RightArm and Spine plan and execute through TEM/JTC on one MuJoCo instance.
- [PASS] TCP/spine TF agrees with state; timed trajectories obey limits and full-state collision checks.
- [PASS] Inactive arm/spine joints remain within 0.003 rad/m of start.
- [PASS] Environment collision fixture is detected and removed without masking checks.
- [PASS] POC orderly shutdown and existing regression tests.

## Known limitations

No public MoveGroup facade, Cartesian, joint pose-goal Gate A/B/C or shared free
spine Gate D claim. World scene is empty apart from the collision fixture; scene
and physical observation synchronization belong to Phase 5. No Octomap updater is
configured, so MoveIt logs the missing 3D sensor plugin. Camera mount placement
remains the description's documented provisional placement.

Unresolved issues: no Phase 2A blocker. Proceed to Phase 2B; no later-phase
capability is inferred from this PASS.
