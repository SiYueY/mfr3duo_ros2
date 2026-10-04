"""Installed configuration agrees with the real URDF and execution endpoints."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory

share = Path(get_package_share_directory('mfr3duo_moveit'))
description = Path(get_package_share_directory('mfr3duo_description'))
robot = ET.parse(description / 'urdf/mfr3duo.urdf').getroot()
srdf = ET.parse(share / 'config/mfr3duo.srdf').getroot()
joints = {j.get('name'): j for j in robot.findall('joint')}
groups = {g.get('name'): g for g in srdf.findall('group')}
required = {'left_arm', 'right_arm', 'spine', 'dual_arm', 'left_arm_spine',
            'right_arm_spine', 'dual_arm_spine'}
assert required <= groups.keys()
assert set(groups) == required | {'left_gripper', 'right_gripper'}
for side in ('left', 'right'):
    chain = groups[side + '_arm'].find('chain')
    assert chain.attrib == {'base_link': side + '_fr3v2_1_link0',
                            'tip_link': side + '_fr3v2_1_hand_tcp'}
    chain = groups[side + '_arm_spine'].find('chain')
    assert chain.get('base_link') == 'franka_spine'
    assert chain.get('tip_link') == side + '_fr3v2_1_hand_tcp'
limits = yaml.safe_load((share / 'config/joint_limits.yaml').read_text())['joint_limits']
assert len(limits) == 15
for name, value in limits.items():
    assert value['max_velocity'] <= float(joints[name].find('limit').get('velocity'))
    assert value['has_acceleration_limits'] and value['max_acceleration'] > 0
configuration = yaml.safe_load((share / 'config/moveit_controllers.yaml').read_text())
assert configuration['moveit_controller_manager'] == 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
controllers = configuration['moveit_simple_controller_manager']
for side in ('left', 'right', 'spine'):
    name = side + '_arm_controller' if side != 'spine' else 'spine_controller'
    entry = controllers[name]
    assert entry['type'] == 'FollowJointTrajectory'
    assert entry['action_ns'] == 'follow_joint_trajectory'
    expected = {side + '_fr3v2_1_joint' + str(i) for i in range(1, 8)} if side != 'spine' else {'franka_spine_vertical_joint'}
    assert set(entry['joints']) == expected
# Physical visible accessories must have collision geometry. Empty sensor frames
# and steering frames have no physical visual and are exempt.
for link in robot.findall('link'):
    if link.find('visual') is not None:
        assert link.find('collision') is not None, link.get('name')
print('Installed SRDF, limits, controllers and physical accessory geometry PASS')
