"""SIGINT while real Robot readiness waits for absent Control must exit promptly."""
import os
from pathlib import Path
import signal
import subprocess
import time
from ament_index_python.packages import get_package_prefix

environment = dict(os.environ)
environment['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
binary = Path(get_package_prefix('mfr3duo_robot')) / 'lib/mfr3duo_robot/task_demo'
process = subprocess.Popen([str(binary)], env=environment,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
try:
    time.sleep(.5)
    if process.poll() is not None:
        raise RuntimeError('Demo exited before initialization interrupt')
    begin = time.monotonic()
    process.send_signal(signal.SIGINT)
    output, _ = process.communicate(timeout=3)
    elapsed = time.monotonic() - begin
    if process.returncode != 130 or 'ROBOT_DEMO_INTERRUPTED' not in output or 'ROS context shut down during Control initialization' not in output:
        raise RuntimeError(output)
    print(output)
    print(f'DEMO_INITIALIZATION_SHUTDOWN_PASS elapsed={elapsed:.3f}s')
finally:
    if process.poll() is None:
        process.kill()
        process.communicate(timeout=2)
