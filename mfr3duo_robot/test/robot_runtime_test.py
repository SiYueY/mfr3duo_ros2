"""Level B proof through production observer, Control, MoveGroup and scene APIs."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from ament_index_python.packages import get_package_share_directory
os.environ['ROS_DOMAIN_ID']=str(100+os.getpid()%100)
scene=Path(get_package_share_directory('mfr3duo_description'))/'mjcf/manipulation.xml'
with tempfile.NamedTemporaryFile(mode='w+',prefix='mfr3duo-phase5-physical-',suffix='.log') as log:
    print('Runtime log:',log.name,flush=True);processes=[]
    try:
        for package,launch,args in [
            ('mfr3duo_control','control.launch.py',['viewer_enabled:=false','model_path:='+str(scene),'initial_keyframe:=manipulation_home','grasp_objects:=box=grasp_object_box=grasp_object_box_collision']),
            ('mfr3duo_moveit','moveit.launch.py',['viewer_enabled:=false','start_control:=false']),
            ('mfr3duo_nav','nav.launch.py',['viewer_enabled:=false','start_control:=false'])]:
            processes.append(subprocess.Popen(['ros2','launch',package,launch]+args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        client=subprocess.Popen(['ros2','run','mfr3duo_robot','robot_runtime_probe'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(client)
        deadline=time.monotonic()+300
        while client.poll() is None and time.monotonic()<deadline:
            assert all(p.poll() is None for p in processes[:-1]),'shared launch exited'
            time.sleep(.1)
        log.seek(0);output=log.read()
        if client.poll()!=0 or not all(marker in output for marker in ('ROBOT_PHYSICAL_PICK_PASS', 'ROBOT_PHYSICAL_PLACE_PASS', 'ROBOT_OWNER_DESTRUCTION_PASS', 'ROBOT_TERMINATION_UNKNOWN_PASS', 'ROBOT_GRASP_LOST_PASS', 'ROBOT_RUNTIME_PASS')):raise RuntimeError(output)
        print(output)
    finally:
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
