"""Exercise the installed ros2_control plugin, controllers, and sensor bridge."""

import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    # Keep this integration test separate from any interactive ROS session.
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "mfr3duo_hardware", "mujoco_control.launch.py",
             "viewer_enabled:=false"],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = rclpy.create_node("mfr3duo_runtime_integration")
        state = {}
        sensor_counts = {"lidar": 0, "camera": 0, "camera_info": 0, "imu": 0}

        def on_joints(message):
            state.update({name: (message.position[index], message.velocity[index])
                          for index, name in enumerate(message.name)})

        def on_lidar(message):
            sensor_counts["lidar"] = len(message.ranges)

        def on_camera(message):
            sensor_counts["camera"] = len(message.data)

        def on_camera_info(message):
            if (message.width == 320 and message.height == 180 and
                    message.k[0] > 0 and message.r[0] == 1.0 and
                    message.header.frame_id.endswith("optical_frame")):
                sensor_counts["camera_info"] += 1

        def on_imu(message):
            if math.isfinite(message.orientation.w):
                sensor_counts["imu"] += 1

        subscriptions = [
            node.create_subscription(JointState, "/joint_states", on_joints, 10),
            node.create_subscription(LaserScan, "/sensors/lidar_front/scan", on_lidar, 10),
            node.create_subscription(Image, "/sensors/front_color/image_raw",
                                     on_camera, qos_profile_sensor_data),
            node.create_subscription(CameraInfo, "/sensors/front_depth/camera_info",
                                     on_camera_info, qos_profile_sensor_data),
            node.create_subscription(Imu, "/imu_broadcaster/imu", on_imu, 10),
        ]
        drive = node.create_publisher(Float64MultiArray,
                                      "/tmr_drive_controller/commands", 10)
        steering = node.create_publisher(Float64MultiArray,
                                         "/tmr_steering_controller/commands", 10)
        spine = node.create_publisher(Float64MultiArray,
                                      "/spine_controller/commands", 10)
        arms = {
            side: node.create_publisher(JointTrajectory,
                                        f"/{side}_arm_controller/joint_trajectory", 10)
            for side in ("left", "right")
        }
        try:
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if process.poll() is not None:
                    raise RuntimeError("ros2_control launch exited early")
                if ("tmrv0_2_joint_1" in state and
                        "franka_spine_vertical_joint" in state and
                        all(sensor_counts.values())):
                    break
            else:
                raise RuntimeError("hardware or sensor topics did not start")

            baseline = state["tmrv0_2_joint_1"][0]
            target = Float64MultiArray()
            target.data = [2.0, 2.0]
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                drive.publish(target)
                rclpy.spin_once(node, timeout_sec=0.05)
                if state["tmrv0_2_joint_1"][0] > baseline + 0.05:
                    break
            else:
                raise RuntimeError("TMR controller command did not move the wheel")
            stop = Float64MultiArray()
            stop.data = [0.0, 0.0]
            drive.publish(stop)

            steering_baseline = state["tmrv0_2_joint_0"][0]
            steer = Float64MultiArray()
            steer.data = [steering_baseline + 0.15, state["tmrv0_2_joint_2"][0]]
            steering.publish(steer)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if state["tmrv0_2_joint_0"][0] > steering_baseline + 0.04:
                    break
            else:
                raise RuntimeError("TMR steering controller did not move")

            for side in ("left", "right"):
                names = [f"{side}_fr3v2_1_joint{number}" for number in range(1, 8)]
                baseline = state[names[0]][0]
                trajectory = JointTrajectory()
                trajectory.joint_names = names
                point = JointTrajectoryPoint()
                point.positions = [state[name][0] for name in names]
                point.positions[0] += 0.04
                point.time_from_start.sec = 2
                trajectory.points = [point]
                arms[side].publish(trajectory)
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if state[names[0]][0] > baseline + 0.02:
                        break
                else:
                    raise RuntimeError(f"{side} arm trajectory did not move")

            spine_baseline = state["franka_spine_vertical_joint"][0]
            height = Float64MultiArray()
            height.data = [spine_baseline + 0.02]
            spine.publish(height)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if state["franka_spine_vertical_joint"][0] > spine_baseline + 0.01:
                    break
            else:
                raise RuntimeError("spine position controller did not move")
            print("ROS integration: arms, spine, TMR, IMU, Camera, LiDAR passed")
        finally:
            subscriptions.clear()
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            if process.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                print("\n".join(log.read().splitlines()[-35:]))


if __name__ == "__main__":
    main()
