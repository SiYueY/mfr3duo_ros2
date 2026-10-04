"""Encoder odometry against ground truth from the same hardware instance."""
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import TimeReference
from tf2_msgs.msg import TFMessage


def stamp(value):
    return value.sec + value.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rate = int(sys.argv[1])
    slow = len(sys.argv) > 2 and sys.argv[2] == 'slow'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    with tempfile.NamedTemporaryFile(mode='w+', prefix='mfr3duo-phase4-odom-', suffix='.log') as log:
        print('Runtime log:', log.name, flush=True)
        process = subprocess.Popen(
            ['ros2', 'launch', 'mfr3duo_control', 'control.launch.py',
             'viewer_enabled:=false', f'controller_update_rate:={rate}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('odometry_runtime_probe')
        latest = {}
        odometry = []
        transforms = []
        subscriptions = []

        def receive_odom(msg):
            latest['odom'] = msg
            odometry.append(msg)

        subscriptions.append(node.create_subscription(Odometry, '/tmr_controller/odom', receive_odom, 50))
        for key, cls, topic in [
            ('truth', PoseStamped, '/sensors/simulation/base_pose'),
            ('reference', TimeReference, '/sensors/simulation/time_reference'),
        ]:
            subscriptions.append(node.create_subscription(cls, topic, lambda msg, k=key: latest.__setitem__(k, msg), 10))
        subscriptions.append(node.create_subscription(TFMessage, '/tf', lambda msg: transforms.extend(t for t in msg.transforms if t.child_frame_id == 'base_link' and t.header.frame_id == 'odom'), 50))
        publisher = node.create_publisher(Twist, '/tmr_controller/cmd_vel', 10)
        hardware_pid = None

        def run(seconds, command=None):
            deadline = time.monotonic() + seconds
            next_pause = time.monotonic() + .2
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('control launch exited')
                if command is not None:
                    publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=.01)
                if slow and time.monotonic() >= next_pause:
                    # Deliberately reduce physics/wall-time ratio without changing
                    # the model timestep or the encoder estimator configuration.
                    os.kill(hardware_pid, signal.SIGSTOP)
                    try:
                        time.sleep(.18)
                    finally:
                        os.kill(hardware_pid, signal.SIGCONT)
                    # The control manager catches up missed iterations when
                    # resumed. Limit its runnable duty sufficiently to make
                    # physics genuinely slower, and assert the measured ratio.
                    next_pause = time.monotonic() + .005

        def check_pose(label):
            a = latest['odom'].pose.pose
            b = latest['truth'].pose
            distance = math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)
            angle = abs(math.remainder(yaw(a.orientation) - yaw(b.orientation), 2 * math.pi))
            assert distance <= .05 and angle <= .06, (label, distance, angle)
            assert latest['odom'].header.frame_id == 'odom'
            assert latest['odom'].child_frame_id == 'base_link'
            assert latest['truth'].header.frame_id == 'simulation_world'
            assert latest['reference'].source == 'same_instance_mujoco'
            assert abs(stamp(latest['odom'].header.stamp) - stamp(latest['truth'].header.stamp)) < .3
            assert all(math.isfinite(v) for v in latest['odom'].pose.covariance)
            print(label, 'position error', distance, 'yaw error', angle, flush=True)

        try:
            deadline = time.monotonic() + 70
            while (len(latest) < 3 or not transforms) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            assert len(latest) == 3 and transforms, 'odom/truth/time-reference/TF absent'
            log.seek(0)
            matched = re.search(r'\[ros2_control_node-\d+\]: process started with pid \[(\d+)\]', log.read())
            assert matched, 'own hardware process identity absent from launch log'
            hardware_pid = int(matched.group(1))
            run(1, Twist())
            ref0 = latest['reference']
            # Normal run covers all four motions; the deliberately CPU-starved
            # run isolates displacement/time scaling with aligned forward wheels.
            motions = [('forward', .1, 0., 0.)] if slow else [
                ('forward', .1, 0., 0.), ('lateral', 0., .1, 0.),
                ('diagonal', .07, .07, 0.), ('rotation', 0., 0., .15)]
            for name, vx, vy, wz in motions:
                cmd = Twist()
                cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
                mark = len(odometry)
                run(12 if slow else 6, cmd)
                samples = odometry[mark:]
                component = (lambda m: m.twist.twist.angular.z) if name == 'rotation' else (
                    (lambda m: m.twist.twist.linear.y) if name == 'lateral' else (lambda m: m.twist.twist.linear.x))
                assert len(samples) > 20 and max(abs(component(m)) for m in samples) > .005, name
                run(1, Twist())
                check_pose(name)
            ref1 = latest['reference']
            ratio = (stamp(ref1.time_ref) - stamp(ref0.time_ref)) / (stamp(ref1.header.stamp) - stamp(ref0.header.stamp))
            assert 0 < ratio < 1.3, ratio
            if slow:
                assert ratio < .85, ('slowdown not observed', ratio)
            # With no fresh command, both measured encoder motion and ROS twist stop.
            run(2)
            assert abs(latest['odom'].twist.twist.linear.x) < .005
            assert abs(latest['odom'].twist.twist.linear.y) < .005
            assert abs(latest['odom'].twist.twist.angular.z) < .01
            # Every TF sample comes from the same estimator, with the same timestamp.
            by_stamp = {(m.header.stamp.sec, m.header.stamp.nanosec): m for m in odometry}
            pairs = [(t, by_stamp[(t.header.stamp.sec, t.header.stamp.nanosec)]) for t in transforms
                     if (t.header.stamp.sec, t.header.stamp.nanosec) in by_stamp]
            assert len(pairs) > 20
            for tf, odom in pairs:
                assert abs(tf.transform.translation.x - odom.pose.pose.position.x) < 1e-12
                assert abs(tf.transform.translation.y - odom.pose.pose.position.y) < 1e-12
                assert abs(tf.transform.rotation.z - odom.pose.pose.orientation.z) < 1e-12
            print(f'ODOMETRY_RUNTIME_PASS rate={rate} slow={slow} physics/wall={ratio}', flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                if hardware_pid is not None:
                    try:
                        os.kill(hardware_pid, signal.SIGCONT)
                    except ProcessLookupError:
                        pass
                os.killpg(process.pid, signal.SIGCONT)
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            if sys.exc_info()[0] is not None:
                log.seek(0)
                print('\n'.join(log.read().splitlines()[-50:]))


if __name__ == '__main__':
    main()
