"""Audit installed, unmodified MoveIt using its public execution interfaces.

No project controller plugin, capability, facade, hardware or simulator is loaded.
Three asynchronous FJT fixtures publish a fixed valid home state. This tests
execution/cancellation protocol only, not physical robot stopping.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


def wait(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise RuntimeError('audit deadline exceeded')


def run(output):
    share = Path(get_package_share_directory('mfr3duo_moveit'))
    hardware = Path(get_package_share_directory('mfr3duo_hardware'))
    description = subprocess.check_output([
        'xacro', str(hardware / 'ros2_control/mfr3duo.ros2_control.xacro'),
        'viewer_enabled:=false'], text=True)
    semantic = (share / 'config/mfr3duo.srdf').read_text()
    home = {joint.attrib['name']: float(joint.attrib['value']) for joint in
            ET.fromstring(semantic).find("group_state[@name='home'][@group='dual_arm_spine']")}
    names = [joint.attrib['name'] for joint in ET.fromstring(description).findall('joint')
             if joint.attrib['type'] != 'fixed']
    values = [home.get(name, 0.035 if 'finger_joint' in name else 0.0) for name in names]
    load = lambda name: yaml.safe_load((share / 'config' / name).read_text())
    params = load('moveit_controllers.yaml')
    params.update({
        'robot_description': description, 'robot_description_semantic': semantic,
        'robot_description_kinematics': load('kinematics.yaml'),
        'robot_description_planning': load('joint_limits.yaml'),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': load('ompl_planning.yaml'), 'use_sim_time': False,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'capabilities': '', 'disable_capabilities': '',
        'allow_trajectory_execution': True, 'moveit_manage_controllers': False,
    })
    # The fixture deliberately controls completion; watchdog cancellation must
    # not masquerade as cancellation caused by the client under test.
    params['trajectory_execution']['execution_duration_monitoring'] = False
    rclpy.init()
    node = rclpy.create_node('upstream_execution_audit')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    lock = threading.Lock()
    records = []
    current_case = {'stagger': False}
    servers = []
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    stop = node.create_publisher(String, '/trajectory_execution_event', 10)

    async def execute(goal):
        with lock:
            delay = 0.15 + (0.2 * (len(records) % 3) if current_case['stagger'] else 0.0)
            record = {'goal': goal, 'future': Future(), 'accepted': time.monotonic(),
                      'cancel': None, 'terminal': False, 'cancel_delay': delay}
            records.append(record)
        await record['future']
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        if goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.succeed()
        with lock:
            record['terminal'] = True
        return result

    def cancel(goal):
        with lock:
            for record in records:
                if bytes(record['goal'].goal_id.uuid) == bytes(goal.goal_id.uuid):
                    record['cancel'] = time.monotonic()
        return CancelResponse.ACCEPT

    for name in params['moveit_simple_controller_manager']['controller_names']:
        servers.append(ActionServer(node, FollowJointTrajectory,
                                    name + '/follow_joint_trajectory', execute,
                                    cancel_callback=cancel))

    def tick():
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name, message.position = names, values
        message.velocity = [0.0] * len(names)
        publisher.publish(message)
        now = time.monotonic()
        with lock:
            for record in records:
                if record['future'].done():
                    continue
                ready = (record['cancel'] is not None and
                         record['goal'].is_cancel_requested and
                         now - record['cancel'] >= record['cancel_delay'])
                if ready or now - record['accepted'] >= 2.0:
                    record['future'].set_result(True)

    timer = node.create_timer(0.01, tick)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    process = None
    results = []
    report_written = False
    try:
        with tempfile.TemporaryDirectory(prefix='mfr3duo-upstream-audit-') as temporary:
            parameter_file = Path(temporary) / 'move_group.yaml'
            parameter_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            log_path = output.with_suffix('.move_group.log')
            with log_path.open('w') as log:
                process = subprocess.Popen([
                    '/opt/ros/humble/lib/moveit_ros_move_group/move_group',
                    '--ros-args', '--params-file', str(parameter_file)],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
                if not client.wait_for_server(timeout_sec=30):
                    raise RuntimeError('official execute_trajectory server unavailable')
                time.sleep(0.5)
                wait(lambda: stop.get_subscription_count() > 0)
                for mode in ('normal', 'action_cancel', 'stop_event',
                             'stop_event_staggered', 'normal_after_stop'):
                    with lock:
                        offset = len(records)
                        current_case['stagger'] = mode == 'stop_event_staggered'
                    goal = ExecuteTrajectory.Goal()
                    path = goal.trajectory.joint_trajectory
                    path.joint_names = list(home)
                    first = JointTrajectoryPoint()
                    first.positions = list(home.values())
                    first.velocities = [0.0] * len(home)
                    last = JointTrajectoryPoint()
                    last.positions = list(home.values())
                    last.velocities = [0.0] * len(home)
                    last.time_from_start.sec = 3
                    path.points = [first, last]
                    sent = client.send_goal_async(goal)
                    wait(sent.done)
                    handle = sent.result()
                    if not handle.accepted:
                        raise RuntimeError('official parent goal rejected')
                    terminal = handle.get_result_async()
                    wait(lambda: len(records) - offset == 3)
                    started = time.monotonic()
                    cancellation = None
                    if mode == 'action_cancel':
                        cancellation = handle.cancel_goal_async()
                    elif mode.startswith('stop_event'):
                        stop.publish(String(data='stop'))
                    ack_seconds = None
                    if cancellation is not None:
                        wait(cancellation.done)
                        ack_seconds = time.monotonic() - started
                    wait(terminal.done)
                    wrapped = terminal.result()
                    with lock:
                        children = records[offset:]
                        canceled_children = sum(record['cancel'] is not None for record in children)
                        terminal_children = sum(record['terminal'] for record in children)
                    result = {'mode': mode, 'parent_status': wrapped.status,
                              'moveit_error': wrapped.result.error_code.val,
                              'child_cancel_count': canceled_children,
                              'child_terminal_count_at_parent_result': terminal_children,
                              'cancel_ack_seconds': ack_seconds,
                              'elapsed_seconds': time.monotonic() - started,
                              'cancel_return_code': cancellation.result().return_code
                              if cancellation is not None else None}
                    if mode.startswith('normal') and (wrapped.status != 4 or wrapped.result.error_code.val != 1):
                        raise RuntimeError('official normal execution failed: ' + str(result))
                    results.append(result)
                    print(json.dumps(result), flush=True)
                    wait(lambda: all(record['terminal'] for record in records[offset:]))
                # Verify the selected shared library belongs to the system prefix.
                maps = Path(f'/proc/{process.pid}/maps').read_text()
                libraries = sorted({line.split()[-1] for line in maps.splitlines()
                                    if 'moveit_simple_controller_manager' in line})
                if not libraries or any(not name.startswith('/opt/ros/humble/') for name in libraries):
                    raise RuntimeError('official plugin provenance failed: ' + str(libraries))
                if 'mfr3duo_moveit_terminal_controllers' in maps or 'mfr3duo_moveit_capabilities' in maps:
                    raise RuntimeError('project compatibility library loaded into audit')
                output.write_text(json.dumps({'plugin_libraries': libraries,
                    'scope': 'software FJT fixture; no physical motion',
                    'results': results}, indent=2) + '\n')
                report_written = True
                print('UPSTREAM_EXECUTION_AUDIT_COMPLETE ' + str(output), flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if process is not None:
            print('OFFICIAL_MOVE_GROUP_SHUTDOWN_EXIT ' + str(process.returncode), flush=True)
            if report_written:
                report = json.loads(output.read_text())
                report['move_group_shutdown_exit'] = process.returncode
                output.write_text(json.dumps(report, indent=2) + '\n')
        executor.shutdown(timeout_sec=3)
        spinner.join(timeout=3)
        timer.cancel()
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('/tmp/mfr3duo-upstream-audit.json'))
    arguments = parser.parse_args()
    run(arguments.output)
