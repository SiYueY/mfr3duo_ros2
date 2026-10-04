"""Freeze navigation configuration against physical walls, sensors and posture."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

share = Path(get_package_share_directory('mfr3duo_nav'))
description = Path(get_package_share_directory('mfr3duo_description'))
control = Path(get_package_share_directory('mfr3duo_control'))
scene = ET.parse(description / 'mjcf/navigation.xml').getroot()
assert scene.find('include').get('file') == 'scene.xml'
rectangles = []
for geom in scene.findall('worldbody/geom'):
    assert geom.get('type') == 'box' and geom.get('group') == '1'
    position = [float(v) for v in geom.get('pos').split()]
    half = [float(v) for v in geom.get('size').split()]
    rectangles.append((position[0], position[1], half[0], half[1]))
assert len(rectangles) == 5
map_config = yaml.safe_load((share / 'maps/navigation.yaml').read_text())
pgm = (share / 'maps' / map_config['image']).read_bytes()
magic, size, maximum, pixels = pgm.split(b'\n', 3)
assert magic == b'P5' and maximum == b'255'
width, height = [int(v) for v in size.split()]
assert len(pixels) == width * height
resolution = map_config['resolution']
ox, oy, angle = map_config['origin']
assert angle == 0 and map_config['negate'] == 0
for row in range(height):
    y = oy + (height-row-.5)*resolution
    for col in range(width):
        x = ox + (col+.5)*resolution
        occupied = any(abs(x-cx) <= hx+1e-9 and abs(y-cy) <= hy+1e-9 for cx,cy,hx,hy in rectangles)
        assert pixels[row*width+col] == (0 if occupied else 254), (row,col)
params = yaml.safe_load((share / 'config/nav2.yaml').read_text())

def verify_time(value):
    if isinstance(value, dict):
        for key, child in value.items():
            if key == 'use_sim_time':
                assert child is False
            else:
                verify_time(child)
verify_time(params)
amcl = params['amcl']['ros__parameters']
assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
assert amcl['scan_topic'] == '/sensors/lidar_front/scan'
assert [amcl[k] for k in ['base_frame_id','odom_frame_id','global_frame_id']] == ['base_link','odom','map']
controller = params['controller_server']['ros__parameters']
dwb = controller['FollowPath']
assert dwb['plugin'] == 'dwb_core::DWBLocalPlanner'
assert dwb['vy_samples'] > 0 and dwb['min_vel_y'] < 0 < dwb['max_vel_y']
tmr = yaml.safe_load((control / 'config/controllers.yaml').read_text())['tmr_controller']['ros__parameters']
assert dwb['max_speed_xy'] <= tmr['max_linear_velocity']
assert dwb['max_vel_theta'] <= tmr['max_angular_velocity']
for axis in ['x','y']:
    assert 0 < dwb['acc_lim_'+axis] <= tmr['linear_acceleration']
    assert -tmr['linear_acceleration'] <= dwb['decel_lim_'+axis] < 0
assert 0 < dwb['acc_lim_theta'] <= tmr['angular_acceleration']
links = {link.get('name') for link in ET.parse(description / 'urdf/mfr3duo.urdf').getroot().findall('link')}
for name in ['local_costmap','global_costmap']:
    costmap = params[name][name]['ros__parameters']
    assert 'robot_radius' not in costmap
    footprint = yaml.safe_load(costmap['footprint'])
    assert len(footprint) == 4
    layer = costmap['obstacle_layer']
    assert set(layer['observation_sources'].split()) == {'front_scan','rear_scan'}
    for side in ['front','rear']:
        scan = layer[side+'_scan']
        assert scan['topic'] == f'/sensors/lidar_{side}/scan'
        assert scan['sensor_frame'] in links
        assert scan['marking'] and scan['clearing']
for key in ['bt_navigator','controller_server','velocity_smoother']:
    assert params[key]['ros__parameters']['odom_topic'] == '/tmr_controller/odom'
posture = yaml.safe_load((share / 'config/navigation_posture.yaml').read_text())
qpos = [float(v) for v in ET.parse(description / 'mjcf/mfr3duo.xml').getroot().find("keyframe/key[@name='transport']").get('qpos').split()]
assert posture['spine_height'] == qpos[16]
assert posture['left_arm'] == qpos[17:24] and posture['right_arm'] == qpos[26:33]
assert posture['gripper_width'] == qpos[24]+qpos[25] == qpos[33]+qpos[34]
print('Physical scene/static map, omni AMCL, two scans, DWB limits, wall time and transport configuration PASS')
