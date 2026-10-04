"""Repeatable Phase 2B prototype runner; exploration is not Gate acceptance."""
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'benchmark'
    assert mode in ('explore', 'benchmark', 'acceptance', 'cancel')
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    marker = 'GATE2_EXPLORE_COMPLETE' if mode == 'explore' else ('GATE2_PASS' if mode == 'acceptance' else 'GATE2_PLANNING_PASS')
    if mode == 'cancel':
        marker = 'OFFICIAL_STOP_PASS'
    executable = 'official_stop_probe' if mode == 'cancel' else 'joint_planning_probe'
    arguments = ['viewer_enabled:=false', 'run_cancel_probe:=true', 'start_control:=false'] if mode == 'cancel' else ['viewer_enabled:=false', 'run_joint_probe:=true', 'joint_probe_mode:=' + mode]
    with tempfile.NamedTemporaryFile(mode='w+', prefix='mfr3duo-gate2-', suffix='.log') as log:
        print('Prototype log: ' + log.name, flush=True)
        process = subprocess.Popen(['ros2', 'launch', 'mfr3duo_moveit', 'moveit.launch.py'] + arguments, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + (100 if mode in ('explore', 'cancel') else 650)
            while time.monotonic() < deadline:
                log.seek(0)
                output = log.read()
                if 'OFFICIAL_STOP_FAIL' in output or 'GATE2_PROBE_FAIL' in output or 'process has died' in output or process.poll() is not None:
                    raise RuntimeError(output)
                if marker in output and (executable + '-2]: process has finished cleanly' if mode == 'cancel' else executable + '-5]: process has finished cleanly') in output:
                    match = re.search(r'move_group-\d+\]: process started with pid \[(\d+)\]', output)
                    assert match, 'MoveGroup PID missing'
                    maps = Path('/proc/' + match.group(1) + '/maps').read_text()
                    for name in ('libmoveit_simple_controller_manager.so', 'libmoveit_move_group_default_capabilities.so'):
                        paths = {line.split()[-1] for line in maps.splitlines() if name in line}
                        assert paths and all(path.startswith('/opt/ros/humble/') for path in paths), paths
                    assert 'mfr3duo_moveit_terminal_controllers' not in maps
                    assert 'mfr3duo_moveit_capabilities' not in maps
                    print('OFFICIAL_PLUGIN_PROVENANCE_PASS /opt/ros/humble controller manager and default capabilities')
                    print(output)
                    return
                time.sleep(.1)
            raise RuntimeError('Joint prototype deadline exceeded\n' + output)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)


if __name__ == '__main__':
    main()
