# mfr3duo_moveit

ROS 2 Humble / MoveIt 2 planning configuration, public MoveGroup facade and
validated Gate A/B/C combined pose-goal prototypes.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mfr3duo_moveit moveit.launch.py viewer_enabled:=false
```

The default launch includes Control once. If Control is already running, add
`start_control:=false`. `controller_update_rate` remains the single period
parameter. All nodes use wall time. No second Simulation is created by MoveIt.

`run_probe:=true` runs the bounded single-group prototype for left arm, right arm
and spine. It checks installed group dimensions, KDL loading, complete fresh
measured state (including passive chassis joints), TCP/spine TF, bounds,
collision, timed trajectories, TEM execution, final position error and inactive
arm/spine positions. The executable owns its application executor; this is not a
facade's threading contract.

The public headers are `mfr3duo_moveit/move_group.hpp` and `moveit_types.hpp`.
Installed consumers use `find_package(mfr3duo_moveit REQUIRED)` and link
`mfr3duo_moveit::mfr3duo_moveit`. The facade uses an application-supplied node;
the application must spin that node concurrently during blocking operations.
It creates no executor/thread and does not link Control or RobotHardware.
`initialize(timeout)` can obtain descriptions and planning configuration from
the running MoveIt node when the supplied node has no model parameters.

Groups and targets are explicit. Add groups, add targets, call `plan(Plan&)`,
then `execute(const Plan&)`, or use `move()`. Plan exposes only const references
and retains its own group set and start state. Changing the builder does not
change a stored Plan. `stop()` only cancels the facade's current operation;
unknown termination blocks the next operation until its actual result arrives.
Only single-arm Cartesian paths are supported; other arm/spine positions remain
fixed. Partial Cartesian paths are diagnostic and cannot execute.

`run_facade_probe:=true` exercises the facade against actual MuJoCo execution.
`run_action_probe:=true start_control:=false` runs the delayed-action software
fixture. See [Phase 3 report](../docs/phase3-report.md) for validation status.

The seven exact groups are `left_arm`, `right_arm`, `spine`, `left_arm_spine`,
`right_arm_spine`, `dual_arm` and `dual_arm_spine`. Gripper groups only define end
effectors. KDL is configured for the four single-tip arm chains; the spine chains
start at `franka_spine` and contain the actual prismatic joint. Both tool tips are
`<side>_fr3v2_1_hand_tcp`. Dual-arm samples use merged full-state AND validation.

Joint position limits come from the URDF. Configured velocity limits do not exceed
it; acceleration limits are conservative planning values (arm 1 rad/s², spine
0.2 m/s²), with default velocity/acceleration scaling 0.1. Controller mapping sends
arm/spine trajectories through the existing FollowJointTrajectory actions.

SRDF exclusions cover physically connected adjacent links, fixed rigid
assemblies, six explicit wheel/housing self-contact pairs and closed fingers
within each gripper. Other robot pairs and environment collisions stay enabled.
The model test inserts a box at the TCP and requires rejection. Physical sensors
and wrist mounts have collision geometry matching their existing visual model.
Wrist-camera mounts retain their documented provisional placement.

This phase uses an empty world planning scene. Sensor observations and physical
Pick/Place scene synchronization belong to Phase 5. No Octomap updater is
configured; MoveIt logs that no 3D sensor plugin is supplied.

```bash
colcon test --packages-select mfr3duo_moveit --event-handlers console_direct+
colcon test-result --verbose
```

Gate 2 acceptance: `python3 mfr3duo_moveit/test/gate2_test.py acceptance`. Eight
targets, ten seeds each, real joint execution and cancellation are checked against
`config/gate2_manifest.yaml`. `cancel` runs delayed-child-terminal action fixtures.
See [Gate 2 report](../docs/phase2b-report.md).

Execution uses the unmodified official `MoveItSimpleControllerManager` and default
`ExecuteTrajectory` capability. The former `compat/` and custom execution plugin
have been removed. The facade publishes the official `trajectory_execution_event`
`stop` event, which stops the entire MoveGroup execution channel.

The application must exclusively and serially use that channel. Facades in one
ROS Context share a lease and must use the same continuously spun Node. External
clients must not concurrently execute MoveGroup goals or command those trajectory
controllers. This contract does not provide per-client stop isolation.

A parent result alone never releases the lease. The facade obtains standard
Action results for the selected controllers' new UUIDs; stop also requires fresh
joint velocity feedback and 100 ms of measured stationary motion. Missing results
or stop evidence preserve `TerminationUnknown`; late completion can converge.
See [official execution migration](../docs/official-execution-report.md) and
[original upstream audit](../docs/upstream-execution-audit.md).
Free shared spine with both arms remains unsupported (Gate D / V1.1).
