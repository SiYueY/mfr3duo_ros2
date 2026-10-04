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
from ament_index_python.packages import get_package_share_directory
import yaml


def main():
    # Keep this integration test separate from any interactive ROS session.
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    with tempfile.TemporaryFile(mode="w+") as log, tempfile.TemporaryDirectory() as config_dir:
        # Hardware regression owns only broadcaster fixtures, not motion controllers.
        xacro = get_package_share_directory("mfr3duo_hardware") + "/ros2_control/mfr3duo.ros2_control.xacro"
        description = subprocess.check_output(
            ["xacro", xacro, "viewer_enabled:=false", "control_period:=0.002"], text=True)
        params = {"controller_manager": {"ros__parameters": {
            "robot_description": description, "update_rate": 500,
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "imu_broadcaster": {"type": "imu_sensor_broadcaster/IMUSensorBroadcaster"},
        }}, "imu_broadcaster": {"ros__parameters": {"sensor_name": "imu", "frame_id": "imu_imu_sensor_frame"}}}
        filename = config_dir + "/hardware.yaml"
        with open(filename, "w") as config:
            yaml.safe_dump(params, config)
        process = subprocess.Popen(
            ["ros2", "run", "controller_manager", "ros2_control_node", "--ros-args", "--params-file", filename],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        spawner = subprocess.Popen(
            ["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster", "imu_broadcaster",
             "--controller-manager-timeout", "40"], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
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

            assert len(state) >= 21
            print("Hardware-only integration: joint states, IMU, Camera, CameraInfo, LiDAR passed")
        finally:
            if spawner.poll() is None:
                os.killpg(spawner.pid, signal.SIGINT)
            try:
                spawner.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(spawner.pid, signal.SIGKILL)
                spawner.wait()
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
