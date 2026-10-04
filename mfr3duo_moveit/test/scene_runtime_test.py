"""Level A production apply/query service acceptance; no physical grasp claim."""
import os
import signal
import subprocess
import tempfile
import time
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
with tempfile.NamedTemporaryFile(mode='w+', prefix='mfr3duo-phase5-scene-', suffix='.log') as log:
    processes=[]
    try:
        launch=subprocess.Popen(['ros2','launch','mfr3duo_moveit','moveit.launch.py','viewer_enabled:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(launch)
        client=subprocess.Popen(['ros2','run','mfr3duo_moveit','scene_runtime_probe'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(client)
        deadline=time.monotonic()+75
        while client.poll() is None and time.monotonic()<deadline:
            assert launch.poll() is None, 'shared launch exited'
            time.sleep(.1)
        log.seek(0);output=log.read()
        if client.poll()!=0 or 'SCENE_RUNTIME_PASS' not in output:
            raise RuntimeError(output)
        print(output)
    finally:
        for process in processes:
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
