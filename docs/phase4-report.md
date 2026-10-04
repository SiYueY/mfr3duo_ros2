# Phase 4 Report

Status: PASS. Gates 1 and 2 and Phase 3 remain PASS. Phase 5 is next.

## Implementation

`mfr3duo_control/{include/mfr3duo_control/tmr_odometry.hpp,src/tmr_odometry.cpp}`
estimates planar increments from measured drive encoder deltas and measured steering,
using the signed two-module geometry and a conditioned least-squares solution.
It integrates the full SE(2) increment, uses ROS timestamp delta for twist and
reports clock/encoder resets, rank/condition, steering transitions and residuals.
The controller publishes `/tmr_controller/odom`, the sole odom→base_link TF, and
confidence covariance/diagnostics through preallocated realtime publishers.
Watchdog coasting remains part of measured odometry; reactivation preserves pose.
The controller unit test exercises 10,000 timestamped updates without update-thread allocation.

The SDK snapshot contains selected immutable body poses and their actual sampling
time. `mj_step` derived transforms and contacts describe the beginning of its final
integration step; the pose sample timestamp reflects that fact. The MFR3Duo wrapper
and RobotHardware expose a standard-C++ read-only base pose. The existing sensor
adapter publishes it and a simulation TimeReference from the same owned instance,
outside the control loop. These samples are acceptance truth and do not feed odometry or AMCL.
The optional `model_path` launch/hardware option selects the physical navigation scene.

`mfr3duo_nav` now exports `mfr3duo_nav::mfr3duo_nav`, with public
`Navigator` and copyable `NavigationHandle`. Its external node/executor boundary,
one-owner Busy rule, bounded response/navigation/terminal deadlines, exact UUID
cancellation, retained late acceptance and unknown-termination guard follow the
public contract. Readiness continuously checks eight ACTIVE lifecycle nodes,
valid/fresh map and odom, action servers and localized map→base_link TF. It creates
no executor/thread and links neither Control, MoveIt nor RobotHardware.

The installed map matches all five physical boxes in `mjcf/navigation.xml`.
Nav2 uses wall time, OmniMotionModel/front LiDAR localization, front and rear
costmap observations and holonomic DWB. The footprint includes actual transport
posture arm/upper-body collision geometry, with localization clearance. Twirling
penalizes unnecessary rotation during holonomic path following, while RotateToGoal
retains terminal heading alignment. The velocity smoother's final output reaches
`/tmr_controller/cmd_vel`; odometry remains `/tmr_controller/odom`.
Applications must execute `config/navigation_posture.yaml` before navigation.
Carrying objects and arbitrary arm postures are outside this footprint acceptance.

## Related backend correction

The wheel model has damping 5 and friction loss 1, but its velocity feedback gain
is 2. Without feedforward, low velocities produced by closed-loop Nav2 acceleration
limiting could stall; larger targets tracked with substantial steady-state error.
An optional damping/friction compensation flag now adds model feedforward at the
already clamped target before the existing effort/actuator clamps. Its SDK default
is false, retaining legacy behavior; MFR3Duo wheel velocity control opts in.
Zero target has no friction preload. No measured qpos/qvel is clipped or overwritten.
The independent root Swerve adapter uses the same optional correction for its
force-driven wheel motors; native position/velocity actuators retain their behavior.
SDK unit tests verify legacy equilibrium, forward/reverse/zero tracking, force and
velocity limits, gear conversion, and absence of direct measured-state changes.

## Validation

The targeted real Nav2 run passed lateral/forward/diagonal/rotation,
NavigateThroughPoses, physical obstacle detour, localization continuity/error,
unknown frame, Busy, exact cancellation and retained result. Actual maximum
`linear.y` was 0.142857 m/s. Detour goal `(2.85, .4, yaw=0)` finished at
`(2.87638, .332801, yaw=.0942531)` in about 40 s, below the unchanged 120 s deadline.
Every 100 ms, a separating-axis check tested the physical fixed-posture footprint
against the obstacle using same-instance world truth; no intersection was accepted.
The final regression also passed fresh measured odometry stopping after cancel. Its detour finished at
`(2.85907, .338539, yaw=.0985045)`; maximum actual `linear.y` was .127673 m/s.

Normal 500 Hz odometry verified forward, lateral, diagonal and rotation against
same-instance truth: maximum position error .0134292 m and heading error .0492888 rad,
within the frozen .05 m/.06 rad acceptance. The slow 1000 Hz run repeatedly stops
only its own hardware process, verifies simulation/wall ratio .191912 (<.85), and
checks aligned forward motion: position error .0000756424 m, heading .000441359 rad.
The slow case isolates timestamp scaling; it does not claim slow steering transitions
or all four slow motion types were tested. Both cases check frame/stamp/TF consistency,
finite covariance and watchdog stopping.

The configuration test checks every map pixel, sensor frames, velocity/acceleration
limits and transport posture against actual MJCF. Software action tests cover
success/failure/rejection, owned-vs-foreign UUID, ACK without terminal, late response,
unknown blocking, terminal convergence and finite destruction. Installed consumer
checks use only the exported installed CMake target. Description's resource
inventory and load test now include the navigation scene.

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/siyuey/workspace/mfr3duo/deps/romujoco-install:$CMAKE_PREFIX_PATH
colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-phase4-final-roslog
colcon test --parallel-workers 1 --event-handlers console_cohesion+
colcon test-result --verbose
```

Logs: `/tmp/mfr3duo-phase4-final-{build,tests,results}.log`,
`/tmp/mfr3duo-phase4-description-{rerun,tests}.log`, and
`/tmp/mfr3duo-phase4-twirling-runtime.log`.
Prior backend validation: 41 SDK tests and four independent-root MFR3Duo tests passed;
all six hardware API/concurrency/zero-allocation benchmarks passed after feedforward.
The full seven-package build passed. The complete workspace regression, followed by
the corrected description resource/load test rerun, reports **51 tests, 0 errors,
0 failures, 0 skipped**. The first description run rejected the newly added scene
because its explicit inventory had not yet been extended; the correction retains
the existing validation and loads the additional scene. All other packages passed
in that full run, including all five Navigator tests. `docs/plan.md` is unchanged.
