"""Bounded, isolated real MoveIt -> TEM -> JTC -> MuJoCo acceptance."""
import os
import signal
import subprocess
import tempfile
import time


def main():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(['ros2', 'launch', 'mfr3duo_moveit', 'moveit.launch.py',
                                    'viewer_enabled:=false', 'run_probe:=true'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 100
            while time.monotonic() < deadline:
                log.seek(0)
                output = log.read()
                if 'PHASE2A_FAIL' in output or 'process has died' in output or process.poll() is not None:
                    raise RuntimeError(output)
                if 'PHASE2A_PASS' in output and 'planning_probe-5]: process has finished cleanly' in output:
                    print(output)
                    return
                time.sleep(.1)
            raise RuntimeError('Phase 2A deadline exceeded\n' + output)
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
