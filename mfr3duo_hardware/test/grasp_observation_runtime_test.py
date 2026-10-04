"""Exercise the production standard pose topics on the one ros2_control-owned scene."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import PoseStamped

os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
scene = Path(get_package_share_directory('mfr3duo_description')) / 'mjcf/manipulation.xml'
with tempfile.NamedTemporaryFile(mode='w+', prefix='mfr3duo-phase5-observer-', suffix='.log') as log:
    process = subprocess.Popen(['ros2', 'launch', 'mfr3duo_control', 'control.launch.py',
        'viewer_enabled:=false', 'model_path:=' + str(scene),
        'initial_keyframe:=manipulation_home',
        'grasp_objects:=box=grasp_object_box=grasp_object_box_collision'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('grasp_observation_runtime_test')
    bundles = {}
    subscriptions = []
    def receive(hand, field, message):
        stamp = message.header.stamp
        key = (hand, stamp.sec * 1000000000 + stamp.nanosec)
        bundles.setdefault(key, {})[field] = message
        if len(bundles) > 100:
            del bundles[next(iter(bundles))]
    for hand in (0, 1):
        side = 'left' if hand == 0 else 'right'
        for field, topic in [('object_pose', '/perception/objects/box/pose'),
                             ('tool_pose', '/perception/tools/' + side + '/pose')]:
            subscriptions.append(node.create_subscription(PoseStamped, topic,
                lambda message, h=hand, f=field: receive(h, f, message), 10))
    try:
        def query(hand):
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                complete = [(stamp, sample) for (h, stamp), sample in bundles.items()
                            if h == hand and len(sample) == 2]
                if complete:
                    newest_stamp, newest = max(complete, key=lambda entry: entry[0])
                    age = (node.get_clock().now().nanoseconds - newest_stamp) / 1e9
                    if -.1 < age < .3:
                        return newest
            raise AssertionError('coherent standard observation topics missing')
        samples = []
        for i in range(30):
            observation = query(i % 2)
            object_pose = observation['object_pose']
            tool_pose = observation['tool_pose']
            assert all(message.header == object_pose.header for message in observation.values())
            assert object_pose.header.frame_id == 'simulation_world'
            stamp = object_pose.header.stamp
            age = node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
            assert -.1 < age < .3, age
            for pose in [object_pose.pose, tool_pose.pose]:
                assert all(math.isfinite(v) for v in [pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                assert abs(sum(v*v for v in [pose.orientation.x,pose.orientation.y,
                    pose.orientation.z,pose.orientation.w])-1) < 1e-6
            assert abs(object_pose.pose.position.x - .8) < .01
            assert abs(object_pose.pose.position.y - .75) < .01
            assert abs(object_pose.pose.position.z - .985) < .01
            samples.append(stamp.sec * 1000000000 + stamp.nanosec)
            time.sleep(.03)
        assert samples[-1] > samples[0], 'physics snapshot did not advance during pose traffic'
        assert process.poll() is None, 'shared hardware exited'
        print('GRASP_OBSERVATION_RUNTIME_PASS common stamp/frame, actual box/keyframe, both tools, standard messages, advancing physics')
    except BaseException:
        log.seek(0)
        print(log.read())
        raise
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
