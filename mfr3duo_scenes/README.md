# MFR3Duo scenes

Kitchen environment resources are exported from RoboCasa layout 2 / style 2 / seed 0.
`scenes/kitchen/raw` retains the original export and resource manifest. Assets have
their own upstream terms; see `raw/ASSET_SOURCE.md` and the retained upstream license.
The Apache-2.0 package license covers the integration code, not a relicensing of assets.

`environment.xml` fixes accessory placements and maps collision geometry to group
3, visual geometry to group 2. Kitchen geometry actively collides on bit 1 but
does not receive kitchen-to-kitchen collisions; robot and free-object bit-1
collision geometry still contacts it, while bit-2 self-collision envelopes remain
reserved for robot self contacts. The original full articulation is retained in this
source model. Both runtime profiles flatten fixed fixtures into world geometry;
`environment_interactive.xml` retains only one lower drawer and one upper cabinet
door as articulated bodies, reducing physics cost while preserving visual geometry.
Robot, keyframe,
navigation map and manipulation profile are composed from one scene configuration.
Runtime needs neither RoboCasa nor robosuite. Generated model paths are cached and
recreated using the installed asset locations, including isolated colcon installs.

After building and sourcing the workspace:

```bash
ros2 launch mfr3duo_robot robot.launch.py scene:=kitchen viewer_enabled:=true
```

Use `scene:=kitchen_interactive` for bounded simulation fixture actuation. Joint
measurements are published separately on `/simulation/scene/joint_states`.
`SceneJointTask` and roserver's `scene_joint` operation use the Robot SDK task lease,
mechanical limits, ramped targets, fresh measured feedback and confirmed cancellation.
They drive environment actuators; robot arm handle grasping is a separate skill.
All other kitchen joints stay fixed, including faucet and appliance knobs.

The map, spawn and docking poses live in this package. Navigation uses the convex
hull of all robot collision geometry in the transport posture, including both arms,
with 25 mm padding, a 10 mm local costmap and a 25 mm global costmap. Kitchen simulation localization
uses fresh MuJoCo base measurements to correct wheel odometry; AMCL remains the
default for the original tasks scene. Ground-truth mode does not accept an AMCL
initial-pose override and stops publishing corrections when physics data freezes.
`planning_scene.yaml` supplies static fixture collision proxies;
`planning_scene_interactive.yaml` splits moving parts from stationary furniture.
The Robot SDK updates those parts from fresh measured joint positions before
planning and after fixture tasks, and transforms obstacles into the current robot
planning frame after base movement. Collision proxies conservatively enclose the
physical geometry; camera streams retain the detailed visual meshes.

`tools/prepare_kitchen.py` is an offline resource adaptation tool. It needs Python
MuJoCo, NumPy and PyYAML; the runtime composer only needs PyYAML. It regenerates
adapted XML, named initial posture, physical obstacle map and planning proxies
from the preserved export. The grasp fixture contains a real free box on the
counter; its placement is observed from physics rather than overwritten by tasks.

Kitchen pregrasp raises and settles the spine before arm motion. Before each
Cartesian segment, the SDK reprojects furniture, support and the unheld object
using fresh measured poses. The test box is 40 x 40 x 60 mm, with matching inertia,
grasp dimensions and resting/place height 0.95 m on the 0.92 m counter.

Counter work uses 20 mm docking position tolerance and 0.025 rad heading
tolerance. The grasp profile bounds shoulder and elbow joints to retain a
counter-safe IK branch, and uses an 80 mm preplace clearance. All candidate
paths and measured execution states still undergo full collision checks.
