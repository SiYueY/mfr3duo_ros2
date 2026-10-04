# mfr3duo_robot

`Robot` coordinates `NavigateTask`, `PickTask`, `PlaceTask` and serial
`TaskSequence` through Control, MoveGroup, Navigator, PlanningSceneInterface and
time-matched standard object/tool pose topics. It links no Hardware or MuJoCo API.
See [Phase 6 report](../docs/phase6-report.md) and the earlier
[physical grasp report](../docs/phase5-report.md) for validation boundaries.

## Whole robot and physical task demo

From the built ROS workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/mfr3duo-robot-logs
ros2 launch mfr3duo_robot robot.launch.py run_demo:=true
```

The viewer defaults to enabled. This launch starts exactly one controller manager,
one Hardware and one MuJoCo simulation; MoveIt and Nav2 reuse it. It uses wall time,
`manipulation.xml`, the physical `tasks.yaml` map, and `task_home`: base at
(-0.6, 0.7), a 50 g box at (0.72, 0.75, 0.985), and its physical table. AMCL starts
at that configured base pose and uses scans rather than simulator ground truth.

The finite application executes navigation to (-0.35, 0.7), actual grasp and a
10 cm lift, then places the box on its support. `ROBOT_DEMO_PASS` is printed only
after all three Tasks succeed. The viewer and bringup remain available afterward.
For headless execution add `viewer_enabled:=false`; `run_demo:=false` starts just
the shared bringup for another application.

The navigation Task first obtains and measures the configured transport posture.
Pick observes the world object again after navigation, approaches along complete
collision-checked paths and uses Control for the selected gripper. It confirms
the mfr3duo_msgs Grasp result and measured opening tolerance before attaching the object,
then measures the lift and relative stability. Place uses the desired *resting
object pose*, checks resting pose, retreats, confirms the object remains on its support, then
updates the world object. A gripper action result alone cannot prove a grasp or release.

`config/grasp.yaml` records the frozen acceptance: observation age <= 0.3 s,
a 0.5 s verification window, relative position drift <= 5 mm
and orientation drift <= 0.05 rad. `config/robot.yaml` selects the whole-demo
approach posture and TCP offset; these do not change the acceptance limits.

## C++ application ownership

Link the installed target:

```cmake
find_package(mfr3duo_robot REQUIRED)
target_link_libraries(my_app PRIVATE mfr3duo_robot::mfr3duo_robot)
```

Include `mfr3duo_robot/robot.hpp`, or the separate `pick_task.hpp`, `place_task.hpp`,
`navigate_task.hpp`, `task_sequence.hpp`, `task_handle.hpp` and `task_result.hpp`.
Robot is a noncopyable, nonmovable PImpl facade. Task inputs are mutable and not
thread-safe; all `clone()` implementations copy owned nested inputs deeply.

The application supplies a Node and spins it concurrently using a multithreaded
Executor; use at least four workers for synchronous SDK calls. Robot and its
observer create no thread or executor. A complete application is in
[tools/task_demo.cpp](tools/task_demo.cpp). With a spinning Node:

```cpp
using namespace std::chrono_literals;
mfr3duo_robot::Robot robot(node);
auto ready = robot.initialize(90s);
if (!ready) throw std::runtime_error(ready.message);

mfr3duo_robot::PickTask pick("box");
pick.set_timeout(60s);
auto result = robot.execute(pick);  // Deep input snapshot.
if (!result) throw std::runtime_error(result.message);

auto task = std::make_unique<mfr3duo_robot::PlaceTask>("box");
task->set_place_pose(resting_object_pose);
auto handle = robot.start(std::move(task));  // Ownership transfer.
auto placed = handle.wait();
if (!placed) throw std::runtime_error(placed.message);
```

Check every Result. Do not continue after a failed `initialize()` or Pick.
`Manipulator::Auto` uses the left arm in this first configured box policy;
explicit Left/Right select the respective chain.

`initialize(timeout)` confirms Control, MoveIt/scene, Nav2 lifecycle/map/TF and
fresh observation within one total deadline. Robot permits one top-level Task.
Synchronous invalid input and Busy results are retained in the returned Handle;
a default Handle returns `InvalidTask`. Sequence failure, cancellation or unknown
termination prevents all remaining children from starting.

Cancellation addresses only the current Task's owned SDK operation. An
acknowledgment is followed by terminal confirmation. If termination or physical
holding is uncertain, Robot enters Error and rejects new Tasks. A known held box
keeps its attached representation; an actual lost/released box is restored to the
world at its newly observed pose. Recovery requires `initialize()` after actual
termination and fresh physical/scene confirmation.

Destroy Robot before stopping/joining the application Executor, then shut down the
ROS context. Running callbacks retain private resources until their bounded
backend operations end; Handle result records survive owner destruction. A late
completion does not replace an already retained failure.

## Reproducible verification

Run from a sourced workspace; each driver creates and cleans up its own shared
bringup with the viewer disabled:

```bash
python3 mfr3duo_robot/test/robot_demo_test.py
python3 mfr3duo_robot/test/robot_runtime_test.py
```

The first verifies the full navigation/physical manipulation sequence. The second
checks real Pick/Place, Busy/Pending input ownership, active cancellation/deadline,
observation loss, missing terminal delivery, explicit recovery, actual unexpected
release, sequence abortion and active owner destruction. Its observation-loss
proxy changes only observation validity; successful physical checks use actual
Hardware observations. The release fault opens the real gripper through a
test-owned Control client.

The earlier Gate 3 probe remains available from a fresh `manipulation_home` scene:

```bash
python3 mfr3duo_robot/test/physical_grasp_test.py
```

The supported physical fixture is the configured box/table/pad geometry. General
arbitrary-object grasping, automatic arm selection and navigation while carrying
an object have no physical acceptance claim; carried navigation returns
`InvalidTask` because its footprint is not validated in this V1 scene.

The physical table remains 0.24 × 0.24 × 1 m. Its planning representation keeps
the observed center and uses height 0.9998 m, a fixed 0.1 mm surface contact skin
for the fixture's measured compliant resting contact. Pick refreshes support and
object together, including immediately before attachment; Place refreshes the
support. Object/support collision rules and physical acceptance values remain
enabled and unchanged. The whole-demo test checks that representation and the
released object's support-relative scene pose after navigation.

Demo motion speed is configured in config/robot.yaml.

| robot parameter | Demo value | Applies to |
| --- | ---: | --- |
| transit_velocity_scaling | 0.50 | Empty-hand transport posture and pregrasp |
| transit_acceleration_scaling | 0.30 | Empty-hand transport posture and pregrasp |
| velocity_scaling | 0.15 | Cartesian grasp/lift, carrying preplace, place and retreat |
| acceleration_scaling | 0.08 | Cartesian grasp/lift, carrying preplace, place and retreat |

All factors are fractions of the configured joint limits in (0, 1]. Every stage
sets its factors explicitly, so a later pregrasp cannot inherit a previous
Cartesian stage's low speed. The frozen grasp.yaml acceptance profile is
unchanged; robot.yaml supplies demo overrides. Without overrides, transit
factors default to 0.1/0.1 and grasp factors use the profile's 0.05/0.02.
The demo's success marker includes task_seconds, measured after readiness.

Gripper Move/Grasp use mfr3duo_msgs types based on Franka Jazzy definitions.
Perception uses `/perception/objects/<id>/pose` and `/perception/tools/{left,right}/pose`.
The `perception.objects_topic_prefix` and `perception.tools_topic_prefix` parameters
select external producers. The built-in adapter publishes simulation ground truth.
A real vision producer is not implemented or validated here.
