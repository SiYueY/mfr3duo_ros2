"""Assemble a portable environment with the installed, named robot model."""
from __future__ import annotations

import copy
import hashlib
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml


def _absolute_assets(root: ET.Element, directory: Path) -> None:
    for element in root.iter():
        resource = element.get('file')
        if resource:
            path = (directory / resource).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            element.set('file', str(path))


def compose_scene(scene_dir: Path, description_dir: Path, *, interactive: bool = False,
                  output_dir: Path | None = None, environment_file: str | None = None) -> tuple[Path, dict]:
    """Use names to rebuild qpos, never append an old full-model keyframe."""
    config = yaml.safe_load((scene_dir / 'scene.yaml').read_text())
    robot_path = description_dir / 'mjcf/mfr3duo.xml'
    robot = ET.parse(robot_path).getroot()
    # The GPIO contract is a symmetric total width. The exported quadratic
    # q1=q2-q2^2 coupling introduces a millimetre opening error and false stalls.
    for coupling in robot.findall('equality/joint'):
        if coupling.get('name') in ('left_hand_finger_coupling', 'right_hand_finger_coupling'):
            coupling.set('polycoef', '0 1 0 0 0')
    environment_file = environment_file or ('environment_interactive.xml' if interactive else 'environment_static.xml')
    environment = ET.parse(scene_dir / environment_file).getroot()
    objects = ET.parse(scene_dir / 'task_objects.xml').getroot()
    _absolute_assets(robot, robot_path.parent)
    _absolute_assets(environment, scene_dir)
    _absolute_assets(objects, scene_dir)

    robot_key = robot.find("keyframe/key[@name='home']")
    source_qpos = list(map(float, robot_key.get('qpos').split()))
    named_qpos: dict[str, list[float]] = {}
    offset = 0
    for joint in robot.find('worldbody').iter():
        if joint.tag not in ('joint', 'freejoint'):
            continue
        kind = 'free' if joint.tag == 'freejoint' else joint.get('type', 'hinge')
        width = 7 if kind == 'free' else 4 if kind == 'ball' else 1
        named_qpos[joint.get('name')] = source_qpos[offset:offset + width]
        offset += width
    if offset != len(source_qpos):
        raise ValueError('Robot home keyframe and named joints disagree')

    x, y, yaw = config['spawn']
    named_qpos['base_freejoint'] = [x, y, .002, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    posture = config['initial_posture']
    named_qpos['franka_spine_vertical_joint'] = [posture['spine_height']]
    for side in ('left', 'right'):
        for i, value in enumerate(posture[side + '_arm'], 1):
            named_qpos[f'{side}_fr3v2_1_joint{i}'] = [value]
        for i in (1, 2):
            named_qpos[f'{side}_fr3v2_1_finger_joint{i}'] = [posture['gripper_width'] / 2]

    scene_joints = []
    for body in environment.iter('body'):
        for joint in list(body.findall('joint')):
            if not interactive or joint.get('name') not in config['interactive_joints']:
                body.remove(joint)
            else:
                lower, upper = map(float, joint.get('range').split())
                scene_joints.append(f"{joint.get('name')}={lower}={upper}")
                ET.SubElement(environment.find('actuator'), 'motor',
                              name=joint.get('name') + '_scene_motor',
                              joint=joint.get('name'), gear='1',
                              ctrllimited='true', ctrlrange='-80 80')
    root = ET.Element('mujoco', model='mfr3duo_kitchen')
    compiler = copy.deepcopy(robot.find('compiler'))
    compiler.set('inertiafromgeom', 'auto')
    compiler.set('inertiagrouprange', '3 3')
    compiler.attrib.pop('meshdir', None)
    root.append(compiler)
    root.append(copy.deepcopy(robot.find('option')))
    defaults = copy.deepcopy(robot.find('default'))
    kitchen_defaults = ET.SubElement(defaults, 'default', {'class': 'kitchen'})
    ET.SubElement(kitchen_defaults, 'geom', {'density': '1000'})
    root.append(defaults)
    for tag in ('asset', 'worldbody', 'actuator', 'sensor', 'tendon', 'equality', 'contact'):
        section = ET.SubElement(root, tag)
        for component in (environment, robot, objects):
            source = component.find(tag)
            if source is not None:
                section.extend(copy.deepcopy(list(source)))
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', azimuth='90', elevation='-30')
    ET.SubElement(visual, 'map', znear='.001')
    for geom in ('caster_front_left_wheel_collision', 'argo_drive_front_wheel_collision',
                 'caster_rear_right_wheel_collision', 'argo_drive_rear_wheel_collision'):
        ET.SubElement(root.find('contact'), 'pair', geom1='floor_room_g0', geom2=geom,
                      condim='6', friction='1.2 1.2 .005 .001 .001', solref='.01 1',
                      solimp='.9 .95 .001')

    qpos: list[float] = []
    for joint in root.find('worldbody').iter():
        if joint.tag not in ('joint', 'freejoint'):
            continue
        kind = 'free' if joint.tag == 'freejoint' else joint.get('type', 'hinge')
        if joint.get('name') in named_qpos:
            value = named_qpos[joint.get('name')]
        elif kind == 'free':
            body = next(b for b in root.iter('body') if joint in list(b))
            value = list(map(float, body.get('pos', '0 0 0').split())) + [1, 0, 0, 0]
        elif kind == 'ball':
            value = [1, 0, 0, 0]
        else:
            value = [float(joint.get('ref', '0'))]
        qpos.extend(value)
    ET.SubElement(ET.SubElement(root, 'keyframe'), 'key', name='kitchen_home',
                  qpos=' '.join(format(v, '.12g') for v in qpos))
    ET.indent(root)
    content = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    key = hashlib.sha256(content).hexdigest()[:20]
    if output_dir is None:
        cache = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache')))
        output_dir = cache / 'mfr3duo/scenes' / key
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir / 'model.xml'
    if not destination.exists() or destination.read_bytes() != content:
        with tempfile.NamedTemporaryFile(dir=output_dir, delete=False) as file:
            temporary = Path(file.name)
            file.write(content)
        temporary.replace(destination)
    config['initial_keyframe'] = 'kitchen_home'
    config['model_path'] = str(destination)
    config['scene_joints'] = ';'.join(scene_joints)
    return destination, config
