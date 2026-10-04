"""Private application verifies facade against one real control/MoveIt instance."""
import os
import signal
import sys
import subprocess
import tempfile
import time

action = len(sys.argv) > 1 and sys.argv[1] == 'action'
marker = 'MOVEIT_ACTION' if action else 'MOVEIT_FACADE'
arguments = ['viewer_enabled:=false', 'start_control:=false', 'run_action_probe:=true'] if action else ['viewer_enabled:=false', 'run_facade_probe:=true']
clean = 'facade_action_probe-2]: process has finished cleanly' if action else 'facade_runtime_probe-5]: process has finished cleanly'
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
with tempfile.NamedTemporaryFile(mode='w+', prefix='mfr3duo-phase3-', suffix='.log') as log:
    print('Runtime log: ' + log.name, flush=True)
    process = subprocess.Popen(['ros2', 'launch', 'mfr3duo_moveit', 'moveit.launch.py'] + arguments, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 170
        while time.monotonic() < deadline:
            log.seek(0)
            output = log.read()
            if marker + '_FAIL' in output or 'process has died' in output or process.poll() is not None:
                raise RuntimeError(output)
            if marker + '_PASS' in output and clean in output:
                print(output)
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Phase 3 runtime deadline\n' + output)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
