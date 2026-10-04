"""One hardware instance, real MoveGroup transport pose, and real Nav2 goals."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import yaml

os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
share = Path(get_package_share_directory('mfr3duo_nav'))
with tempfile.NamedTemporaryFile(mode='w+', prefix='mfr3duo-phase4-nav-', suffix='.log') as log, tempfile.TemporaryDirectory(prefix='mfr3duo-nav-profile-') as directory:
    print('Runtime log:', log.name, flush=True)
    profile = yaml.safe_load((share / 'config/facade.yaml').read_text())
    profile['/**']['ros__parameters']['navigation_posture'] = yaml.safe_load((share / 'config/navigation_posture.yaml').read_text())
    parameters = Path(directory) / 'profile.yaml'
    parameters.write_text(yaml.safe_dump(profile))
    processes = []
    try:
        for package, launch, args in [
            ('mfr3duo_nav', 'nav.launch.py', ['viewer_enabled:=false']),
            ('mfr3duo_moveit', 'moveit.launch.py', ['viewer_enabled:=false', 'start_control:=false']),
        ]:
            processes.append(subprocess.Popen(['ros2', 'launch', package, launch] + args,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        client = subprocess.Popen(['ros2', 'run', 'mfr3duo_nav', 'navigator_runtime_probe',
            '--ros-args', '--params-file', str(parameters)], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(client)
        deadline = time.monotonic() + 540
        while client.poll() is None and time.monotonic() < deadline:
            if any(p.poll() is not None for p in processes[:-1]):
                raise RuntimeError('shared control/navigation launch exited')
            time.sleep(.1)
        log.seek(0)
        output = log.read()
        if client.poll() != 0 or 'NAVIGATOR_RUNTIME_PASS' not in output:
            raise RuntimeError(output)
        print(output)
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
