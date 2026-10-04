"""Phase 1A real MuJoCo actions, command paths and lifecycle acceptance."""
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from tf2_msgs.msg import TFMessage
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory


def main():
    rate = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'mfr3duo_control', 'control.launch.py',
             'viewer_enabled:=false', f'controller_update_rate:={rate}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('control_runtime_test')
        state = {}
        history = []
        finger_tf = {}
        tf_sub = node.create_subscription(TFMessage, '/tf', lambda msg: finger_tf.update({t.child_frame_id: t for t in msg.transforms}), 20)

        def joints(message):
            state.update({name: (message.position[i], message.velocity[i])
                          for i, name in enumerate(message.name)})
            history.append((time.monotonic(), dict(state)))

        sub = node.create_subscription(JointState, '/joint_states', joints, 10)
        pub = node.create_publisher(Twist, '/tmr_controller/cmd_vel', 1)
        list_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        switch = node.create_client(SwitchController, '/controller_manager/switch_controller')

        def wait(predicate, timeout, label, command=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('controller_manager exited')
                if command is not None:
                    pub.publish(command)
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
            raise RuntimeError(f"{label}; latest TMR={ {name: value for name, value in state.items() if name.startswith('tmr')} }")

        def future(fut, timeout, label):
            wait(fut.done, timeout, label)
            return fut.result()

        def activate(name, enabled):
            request = SwitchController.Request()
            request.activate_controllers = [name] if enabled else []
            request.deactivate_controllers = [] if enabled else [name]
            request.strictness = SwitchController.Request.STRICT
            request.timeout.sec = 5
            assert future(switch.call_async(request), 8, 'switch timeout').ok

        try:
            assert list_client.wait_for_service(timeout_sec=40)
            assert switch.wait_for_service(timeout_sec=10)
            deadline = time.monotonic() + 40
            expected = {'joint_state_broadcaster', 'imu_broadcaster', 'left_arm_controller',
                        'right_arm_controller', 'spine_controller', 'tmr_controller',
                        'left_gripper_controller', 'right_gripper_controller'}
            while time.monotonic() < deadline:
                controllers = future(list_client.call_async(ListControllers.Request()), 5, 'list timeout')
                if expected <= {c.name for c in controllers.controller if c.state == 'active'}:
                    break
            else:
                raise RuntimeError('controllers not ACTIVE')
            wait(lambda: 'right_fr3v2_1_joint7' in state and 'tmrv0_2_joint_3' in state, 10, 'joint states absent')
            root = ET.parse(get_package_share_directory('mfr3duo_description') + '/urdf/mfr3duo.urdf')
            limits = {joint.attrib['name']: joint.find('limit').attrib
                      for joint in root.findall('joint') if joint.find('limit') is not None}

            for side in ('left', 'right', 'spine'):
                names = ([f'{side}_fr3v2_1_joint{i}' for i in range(1, 8)]
                         if side != 'spine' else ['franka_spine_vertical_joint'])
                client = ActionClient(node, FollowJointTrajectory,
                                      f'/{side}_controller/follow_joint_trajectory'
                                      if side == 'spine' else f'/{side}_arm_controller/follow_joint_trajectory')
                assert client.wait_for_server(timeout_sec=10)
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = names
                point = JointTrajectoryPoint()
                point.positions = [state[name][0] for name in names]
                point.positions[0] += .02 if side == 'spine' else .04
                point.time_from_start.sec = 2
                goal.trajectory.points = [point]
                for name, value in zip(names, point.positions):
                    assert float(limits[name]['lower']) <= value <= float(limits[name]['upper'])
                start = time.monotonic()
                mark = len(history)
                handle = future(client.send_goal_async(goal), 5, f'{side} acceptance timeout')
                assert handle.accepted, side
                result = future(handle.get_result_async(), 8, f'{side} result timeout')
                assert result.status == GoalStatus.STATUS_SUCCEEDED, (side, result.status, result.result)
                assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
                elapsed = time.monotonic() - start
                assert 1.8 <= elapsed <= 5.0, (side, elapsed)
                error = max(abs(state[name][0] - target) for name, target in zip(names, point.positions))
                assert error <= (.003 if side == 'spine' else .02), (side, error)
                samples = history[mark:]
                assert samples, side
                for _, sample in samples:
                    for name in names:
                        q, velocity = sample[name]
                        bound = limits[name]
                        assert float(bound['lower']) - 1e-5 <= q <= float(bound['upper']) + 1e-5
                        assert abs(velocity) <= float(bound['velocity']) + .01, (name, velocity)
                # Malformed joint identity is rejected by the real action server.
                goal.trajectory.joint_names[0] = 'unknown_joint'
                rejected = future(client.send_goal_async(goal), 5, 'invalid goal response')
                assert not rejected.accepted
                print(f'{side}: action success, duration={elapsed:.3f}s, error={error:.6f}, limits and invalid goal PASS')
                client.destroy()

            for side in ('left', 'right'):
                client = ActionClient(node, GripperCommand, f'/{side}_gripper_controller/gripper_cmd')
                assert client.wait_for_server(timeout_sec=10)
                finger = f'{side}_fr3v2_1_finger_joint1'
                for position, effort in ((.04, 0.0), (.0, 5.0), (.04, 0.0)):
                    goal = GripperCommand.Goal()
                    goal.command.position, goal.command.max_effort = position, effort
                    handle = future(client.send_goal_async(goal), 5, 'gripper acceptance')
                    assert handle.accepted
                    result = future(handle.get_result_async(), 10, 'gripper result')
                    assert result.status == GoalStatus.STATUS_SUCCEEDED, (side, result)
                    assert result.result.reached_goal and not result.result.stalled, (side, result)
                    assert abs(result.result.position - position) <= .001
                    wait(lambda: finger in state and abs(state[finger][0] - position) < .002, 3, 'finger projection mismatch')
                    wait(lambda: all(f'{side}_fr3v2_1_{suffix}' in finger_tf and abs(finger_tf[f'{side}_fr3v2_1_{suffix}'].transform.translation.y - sign * position) < .002 for suffix, sign in (('leftfinger', 1), ('rightfinger', -1))), 3, 'finger TF absent or stale')
                    for suffix, sign in (('leftfinger', 1), ('rightfinger', -1)):
                        transform = finger_tf[f'{side}_fr3v2_1_{suffix}']
                        assert abs(transform.transform.translation.y - sign * position) < .002, (side, suffix, transform)
                for position, effort in ((-.001, 0.0), (.041, 0.0), (.01, -1.0), (.01, float('nan')), (.01, 101.0)):
                    goal = GripperCommand.Goal()
                    goal.command.position, goal.command.max_effort = position, effort
                    assert not future(client.send_goal_async(goal), 5, 'invalid gripper goal').accepted
                goal = GripperCommand.Goal()
                goal.command.position = 0.0
                handle = future(client.send_goal_async(goal), 5, 'cancel goal acceptance')
                assert handle.accepted
                wait(lambda: state[finger][0] < .035, 3, 'gripper did not start closing')
                assert future(handle.cancel_goal_async(), 3, 'cancel response').goals_canceling
                result = future(handle.get_result_async(), 3, 'cancel terminal result')
                assert result.status == GoalStatus.STATUS_CANCELED
                held = result.result.position
                mark = len(history)
                wait(lambda: len(history) > mark + 100, 3, 'hold samples absent')
                assert .001 < held < .04, (side, held)
                assert abs(state[finger][0] - held) < .002, (side, held, state[finger])
                print(f'{side} gripper: real open/close, default and positive effort, invalid goals, mimic TF, canceled hold PASS')
                client.destroy()

            if len(sys.argv) > 2:
                subprocess.run([sys.argv[2]], check=True, timeout=45)

            drive_names = ('tmrv0_2_joint_1', 'tmrv0_2_joint_3')
            steering_names = ('tmrv0_2_joint_0', 'tmrv0_2_joint_2')
            for label, vx, vy, wz in [('forward', .1, 0, 0), ('lateral', 0, .1, 0),
                                       ('rotation', 0, 0, .3), ('reverse', -.1, 0, 0)]:
                command = Twist()
                command.linear.x, command.linear.y, command.angular.z = float(vx), float(vy), float(wz)
                mark = len(history)

                def aligned_and_moving():
                    for i, (x, y) in enumerate(((.3, -.2), (-.3, .2))):
                        direction = math.atan2(vy + wz*x, vx - wz*y)
                        steering = state[steering_names[i]][0]
                        speed = state[drive_names[i]][1]
                        direction_error = abs(math.remainder(steering-direction, math.pi))
                        if direction_error > .2 or abs(speed) < .3:
                            return False
                        # Wheel reversal must still produce the requested contact-point vector.
                        actual_x = speed * math.cos(steering)
                        actual_y = speed * math.sin(steering)
                        if actual_x * (vx-wz*y) + actual_y * (vy+wz*x) <= 0:
                            return False
                    return True

                wait(aligned_and_moving, 10, f'{label} alignment or movement failed', command)
                assert len(history) > mark
                print(f'TMR {label}: measured alignment and signed wheel motion PASS')
            zero = Twist()
            pub.publish(zero)
            wait(lambda: all(abs(state[n][1]) < .05 for n in drive_names), 4, 'zero did not stop', zero)
            held = [state[n][0] for n in steering_names]
            before = len(history)
            wait(lambda: len(history) >= before + 20, 3, 'hold samples absent', zero)
            assert max(abs(state[n][0]-q) for n, q in zip(steering_names, held)) < .08
            forward = Twist(); forward.linear.x = .1
            wait(lambda: any(abs(state[n][1]) > .3 for n in drive_names), 10, 'watchdog preparation failed', forward)
            # No commands: observe monotonic timeout and actual wheel stop.
            wait(lambda: all(abs(state[n][1]) < .05 for n in drive_names), 4, 'watchdog failed')
            activate('tmr_controller', False)
            activate('tmr_controller', True)
            mark = len(history)
            wait(lambda: len(history) >= mark + 20, 3, 'reactivation samples absent')
            assert all(abs(state[n][1]) < .05 for n in drive_names)
            print(f'Phase 1A runtime {rate}Hz: zero hold, watchdog, deactivate/reactivate PASS')
        finally:
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            log.seek(0)
            if sys.exc_info()[0] is not None:
                print('\n'.join(log.read().splitlines()[-60:]))

if __name__ == '__main__':
    main()
