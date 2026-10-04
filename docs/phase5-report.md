# Phase 5 Report

> 接口迁移说明：下文的自定义 ObserveGrasp 服务和双指接触契约为历史实现。
> 当前使用 mfr3duo_msgs 夹爪动作及标准物体/工具位姿 Topic，见 [迁移报告](msgs-migration-report.md)。


Status: PASS. Gate 3: PASS. Phase 6 public Robot/Task orchestration is next.

## Implemented boundaries

`mfr3duo_interfaces` contains only GraspObservation and ObserveGrasp. The backend
reads object pose, selected TCP pose and contacts from one immutable RobotState
snapshot. Object mappings and selected bodies are configured before initialization.
No MuJoCo pointer or body/geom ID crosses the backend boundary. RobotHardware's
standard C++ interface forwards observations; the existing sensor adapter exposes
the service using the same hardware-owned simulation. Physics sequence freshness,
common pose timestamp/frame, identity validation and bounded service calls are
checked outside the realtime control update.

The private SimulationGraspObserver uses the application's external node/executor.
It rejects unavailable, invalid, stale, incoherent or nonfinite observations.
Frozen acceptance values are maximum age .3 s, service timeout .5 s, sustained
bilateral contact .2 s, stability window .5 s, relative translation drift .005 m
and orientation drift .05 rad. The 50 g box, physical support, 10 cm lift and
precision motion settings are recorded in `mfr3duo_robot/config/grasp.yaml`.

PlanningSceneInterface exports confirmed apply/query, atomic batch validation,
world/attached queries, explicit arm attachment and selected-finger contact rules.
V1 geometry is explicit solid primitives; mesh/plane/subframe operations are
rejected. Attachment retains the two fingers as touch links. Restoring temporary
contact permissions while attached keeps its valid touch rules; confirmed detach
restores the saved world collision entries. Unconfirmed scene application blocks
further operations until pending application completion and fresh scene query.
Scene queries include fixed transforms and compose CollisionObject.pose with its
primitive poses, matching the actual Humble server representation.

## Physical defects corrected

The grasp fixture initially intersected the home arm. Its physical table and box
were moved together to (.8, .75), and manipulation_home explicitly initializes the
added free body. The table is .24 × .24 × 1 m, top at .96 m; the box is
.04 × .04 × .05 m with mass .05 kg. No measured robot or object state is clipped,
teleported or otherwise overwritten during task execution.

Auxiliary `_sc` approximation bodies around the hand extend into the grasp volume.
Treating those approximations as environment geometry blocked the box before the
fingers could reach it, pushing the otherwise fixed spine beyond its unchanged
3 mm inactive-joint limit. Those auxiliary self-collision proxies now use a
separate 2/2 collision mask in both description checkouts. Actual robot collision
meshes, fingers, pads, table, object and environment retain 1/1 collision masks.
The regression checks all 72 proxies and the retained physical hand/pad/environment
geometry. This is separate from MoveIt's selected-finger AllowedCollisionMatrix.
The ineffective spine stiffness experiment was reverted: stiffness remains 5000,
damping 1800. Acceptance limits and the inactive-spine guard were not relaxed.

Gripper blocking detection previously required effort near the configured cap.
A compliant gripper can stall with meaningful contact force below that cap; the
controller now requires sustained low measured velocity, an unreached target and
nonzero measured effort. Its regression explicitly stalls at 1.5 N under a 7.5 N
cap. Stall is still not interpreted as object identity or physical grasp success.

## Verified results

Level A tests cover observation rejection/deadlines/destruction and real MoveIt
scene apply/query, world/attached transitions, atomic invalid input and isolated
finger contact permissions. Mock observations claim software coverage only.

The Level B private step driver uses Control only for gripper commands, MoveGroup
for arm motion and full-fraction Cartesian approach/lift/place/retreat, and the
production service for actual contacts and poses. It runs one control/hardware
instance and a MoveIt launch with start_control=false. The object rose 10 cm;
relative drift was approximately 3.4 mm, within the frozen 5 mm limit. Place
confirmed actual release and support height before scene restoration and retreat.

The failure-recovery driver verifies, after confirmed owned motion termination:

- Actual bilateral contacts and stable relative pose retain the attached object.
- Injected unavailable observation returns Unknown and retains physical/scene
  holding. Unknown motion termination likewise forbids recovery mutations.
- Actual release returns Released, removes the attachment and restores world
  geometry from the new physical observation, then completes the full retreat.

The Unknown observation is an injected software failure while the object is
physically held; the held and released classifications use actual same-instance
physics. The released branch uses explicit gripper release, not an induced free
fall or a general arbitrary-object drop claim. Robot Error/Task error mapping
belongs to Phase 6; this phase exposes the private recovery classification.

Targeted logs: `/tmp/mfr3duo-phase5-recovery-{build,runtime}.log`,
`/tmp/mfr3duo-phase5-observation-runtime.log`,
`/tmp/mfr3duo-phase5-observer-software.log`,
`/tmp/mfr3duo-phase5-scene-origin-runtime.log`, and
`/tmp/mfr3duo-phase5-gripper-test.log`.

```bash
set -e
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/home/siyuey/workspace/mfr3duo/deps/romujoco-install:$CMAKE_PREFIX_PATH
colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-phase5-final-roslog
colcon test --parallel-workers 1 --event-handlers console_cohesion+
colcon test-result --verbose
```

The eight-package build passed. The complete workspace run passed description,
backend, hardware, Control, Nav and Robot tests, including the 80-run Gate 2 and
actual physical recovery. One old single-group probe failed because it compared
independent state/TF samples and created per-group state monitors that could still
contain default joints. It now reads its already-complete fresh shared monitor
and checks TF against the originating JointState timestamp. Original position and
execution tolerances remain unchanged; its targeted rerun passed all three groups.
The latest `colcon test-result --verbose` reports **49 tests, 0 errors, 0 failures,
0 skipped**. That targeted run replaces the MoveIt package's latest CTest result
with one test; the other eight MoveIt tests passed in the preceding complete run
and remain evidenced in the full log. Independent root backend was freshly built
against the current SDK and its four native tests passed.

Final logs: `/tmp/mfr3duo-phase5-final-{build,tests,results}.log`,
`/tmp/mfr3duo-phase5-fresh-state-{build,test}.log`, and
`/tmp/mfr3duo-phase5-root-{configure,build,tests}.log`.

Modified implementation files span the two new interface definitions, backend
`data/grasp.hpp` and simulation read, Hardware options/adapter/sensor service,
Control gripper stall detection, MoveIt PlanningSceneInterface and the private
Robot observer/recovery sources and physical probes. Related description changes
are the manipulation scene, pad names and self-only proxy mask; both description
checkouts received that mask correction. Documentation and all affected package
manifests/build declarations were updated. No unresolved Phase 5 issue remains.
V1 supports the configured primitive box, not arbitrary mesh grasps or carried
navigation. Robot Task Error mapping is reserved for Phase 6.
`docs/plan.md` remains the user's original staged 3427-line document.
