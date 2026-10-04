# mfr3duo_msgs

Three device interfaces copied unchanged from Franka Robotics Jazzy commit
`6cedf7f1a2ca280c433f643eae697be23eb2a15e`: Move, Grasp and GraspEpsilon.
See [NOTICE.md](NOTICE.md), [upstream.json](upstream.json) and [LICENSE](LICENSE).
They build on ROS 2 Humble without franka_msgs or the Franka hardware driver.
The ROS package/type namespace differs, so franka_msgs clients cannot connect directly.

Each gripper controller exposes `~/move`, `~/grasp`, and the existing standard
`~/gripper_cmd`. All three share one device execution lease. JointState retains
measured finger positions. Width/current_width are the total finger separation [m],
speed is total opening/closing speed [m/s], force is [N]. GraspEpsilon defaults to
0.005 m for both inner and outer deviation. Grasp width is expected object size,
not a command to close all the way. Move requires reaching the target opening;
Grasp requires settled closing feedback inside the expected width tolerance.
An empty or wrong-width grasp fails. Device completion alone does not identify
an object or prove lift/transport/release. Simulation evidence is not real hardware validation.

```bash
ros2 action send_goal /left_gripper_controller/move mfr3duo_msgs/action/Move \
  '{width: 0.08, speed: 0.05}' --feedback
ros2 action send_goal /left_gripper_controller/grasp mfr3duo_msgs/action/Grasp \
  '{width: 0.04, epsilon: {inner: 0.005, outer: 0.005}, speed: 0.05, force: 20.0}' --feedback
```

Place an appropriate object between the fingers before issuing Grasp.
Standard object/tool PoseStamped perception topics belong to the sensor pipeline,
not this package. There are no object IDs, camera observations or task results here.
