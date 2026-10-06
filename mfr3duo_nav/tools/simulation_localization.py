#!/usr/bin/env python3
"""MuJoCo-only map/odom correction from measured simulation pose and wheel odom."""
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import TimeReference
from tf2_ros import TransformBroadcaster


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def correction(measured, odometry):
    angle = yaw(measured.orientation) - yaw(odometry.orientation)
    c, s = math.cos(angle), math.sin(angle)
    return (measured.position.x - c * odometry.position.x + s * odometry.position.y,
            measured.position.y - s * odometry.position.x - c * odometry.position.y,
            angle)


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def valid_pose(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return all(math.isfinite(v) for v in values) and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < 1e-3


class SimulationLocalization(LifecycleNode):
    def __init__(self):
        super().__init__('simulation_localization')
        self.odometry = deque(maxlen=100)
        self.measured = None
        self.physics_time = None
        self.physics_received = 0.
        self.odometry_received = 0.
        self.measured_received = 0.
        self.active = False
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/tmr_controller/odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, '/sensors/simulation/base_pose', self.on_pose, 10)
        self.create_subscription(TimeReference, '/sensors/simulation/time_reference', self.on_time, 10)
        self.create_timer(.02, self.publish)
        self.get_logger().info('Localization source: MuJoCo simulation ground truth')

    def on_activate(self, state):
        result = super().on_activate(state)
        self.active = result == TransitionCallbackReturn.SUCCESS
        return result

    def on_deactivate(self, state):
        self.active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.active = False
        self.odometry.clear()
        self.measured = None
        self.physics_time = None
        self.physics_received = 0.
        self.odometry_received = 0.
        self.measured_received = 0.
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        self.active = False
        return super().on_shutdown(state)

    def on_odom(self, message):
        if message.header.frame_id == 'odom' and message.child_frame_id == 'base_link' and valid_pose(message.pose.pose):
            self.odometry.append(message)
            self.odometry_received = time.monotonic()

    def on_pose(self, message):
        if message.header.frame_id == 'simulation_world' and valid_pose(message.pose):
            self.measured = message
            self.measured_received = time.monotonic()

    def on_time(self, message):
        value = message.time_ref.sec * 1000000000 + message.time_ref.nanosec
        if value != self.physics_time:
            self.physics_time = value
            self.physics_received = time.monotonic()

    def publish(self):
        received = time.monotonic()
        if (not self.active or self.measured is None or not self.odometry or
                received - self.physics_received > .5 or
                received - self.odometry_received > .5 or
                received - self.measured_received > .5):
            return
        now = self.get_clock().now()
        measured_stamp = stamp(self.measured)
        age = now.nanoseconds * 1e-9 - measured_stamp
        odometry = min(self.odometry, key=lambda value: abs(stamp(value) - measured_stamp))
        # The controller and simulation-pose publisher use separate executors.
        # Thirty milliseconds is too strict under scheduler jitter and can let
        # map -> odom disappear from TF despite both sources remaining live.
        if age < -.1 or age > .5 or abs(stamp(odometry) - measured_stamp) > .1:
            return
        x, y, angle = correction(self.measured.pose, odometry.pose.pose)
        if not all(math.isfinite(value) for value in (x, y, angle)):
            return
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(angle / 2), math.cos(angle / 2)
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = SimulationLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
