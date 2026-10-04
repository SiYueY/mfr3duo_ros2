"""Verify the combined scene, initial pose, map and one-hardware bringup."""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import yaml
root = Path(get_package_share_directory('mfr3duo_robot'))
description = Path(get_package_share_directory('mfr3duo_description'))
module_spec = importlib.util.spec_from_file_location('robot_launch', root / 'launch/robot.launch.py')
module = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(module)
actions = module.generate_launch_description().entities
includes = [item for item in actions if isinstance(item, IncludeLaunchDescription)]
assert len(includes) == 3
context = LaunchContext()
context.launch_configurations.update(viewer_enabled='false', controller_update_rate='500', run_demo='false')
arguments = [dict(item.launch_arguments) for item in includes]
assert arguments[0]['initial_keyframe'] == 'task_home'
assert arguments[1]['start_control'] == arguments[2]['start_control'] == 'false'
rewritten = Path(arguments[2]['params_file'].perform(context))
try:
    params = yaml.safe_load(rewritten.read_text())
    assert params['amcl']['ros__parameters']['initial_pose']['x'] == -.6
    assert params['amcl']['ros__parameters']['initial_pose']['y'] == .7
    assert params['amcl']['ros__parameters']['use_sim_time'] is False
finally:
    rewritten.unlink()
model = ET.parse(description / 'mjcf/manipulation.xml').getroot()
initial = [float(v) for v in model.find("keyframe/key[@name='task_home']").get('qpos').split()]
assert initial[:2] == [-.6, .7] and initial[35:38] == [.72, .75, .985]
rectangles = []
for scene in (description / 'mjcf/navigation.xml', description / 'mjcf/manipulation.xml'):
    for geom in ET.parse(scene).getroot().findall('worldbody/geom'):
        if geom.get('type') != 'box':
            continue
        pos = [float(v) for v in geom.get('pos').split()]
        half = [float(v) for v in geom.get('size').split()]
        rectangles.append((pos[0], pos[1], half[0], half[1]))
assert len(rectangles) == 6
map_config = yaml.safe_load((root / 'maps/tasks.yaml').read_text())
magic, size, maximum, pixels = (root / 'maps' / map_config['image']).read_bytes().split(b'\n', 3)
width, height = map(int, size.split())
assert magic == b'P5' and maximum == b'255' and len(pixels) == width * height
ox, oy, yaw = map_config['origin']
assert yaw == 0
for row in range(height):
    for col in range(width):
        x = ox + (col + .5) * map_config['resolution']
        y = oy + (height - row - .5) * map_config['resolution']
        occupied = any(abs(x-cx) <= hx+1e-9 and abs(y-cy) <= hy+1e-9 for cx, cy, hx, hy in rectangles)
        assert pixels[row*width+col] == (0 if occupied else 254)
if importlib.util.find_spec('mujoco') is not None:
    import mujoco
    physics = mujoco.MjModel.from_xml_path(str(description / 'mjcf/manipulation.xml'))
    data = mujoco.MjData(physics)
    mujoco.mj_resetDataKeyframe(physics, data, physics.key('task_home').id)
    mujoco.mj_forward(physics, data)
    table, box = physics.geom('grasp_table').id, physics.geom('grasp_object_box_collision').id
    assert all(box in (c.geom1, c.geom2) for c in data.contact if table in (c.geom1, c.geom2))
print('ROBOT_LAUNCH_PASS one control, shared model/time, task key/AMCL pose, exact physical map')
