#!/usr/bin/env python3
"""Offline adaptation of the preserved export; not a RoboCasa runtime dependency."""
from pathlib import Path
import argparse
import copy
import math
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

INTERACTIVE_JOINTS = ['bottom_main_group_1_slidejoint', 'top_main_group_leftdoorhinge']


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def collision_footprint(model, state):
    """Conservative convex hull of every robot collision geom in base coordinates."""
    base = int(model.joint('base_freejoint').bodyid[0])
    rotation = state.xmat[base].reshape(3, 3)
    points = []
    for geom in range(model.ngeom):
        ancestor = int(model.geom_bodyid[geom])
        while ancestor and ancestor != base:
            ancestor = int(model.body_parentid[ancestor])
        if ancestor != base or not model.geom_contype[geom]:
            continue
        for signs in ((x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)):
            world = state.geom_xpos[geom] + state.geom_xmat[geom].reshape(3, 3) @ (
                model.geom_aabb[geom, :3] + model.geom_aabb[geom, 3:] * signs)
            points.append(tuple((rotation.T @ (world - state.xpos[base]))[:2]))
    def cross(origin, a, b):
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])
    points = sorted(set(points))
    halves = []
    for ordered in (points, reversed(points)):
        half = []
        for point in ordered:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    return [[float(x), float(y)] for x, y in halves[0] + halves[1]]


def prepare(directory: Path, robot_dir: Path):
    source = directory / 'raw/scene.xml'
    root = ET.parse(source).getroot()
    absolute = copy.deepcopy(root)
    for e in absolute.iter():
        if e.get('file'):
            e.set('file', str((source.parent / e.get('file')).resolve()))
    model = mujoco.MjModel.from_xml_string(ET.tostring(absolute, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # Preserve inferred masses before changing geom groups or composing defaults.
    for body in root.iter('body'):
        if body.find('joint') is None:
            continue
        i = model.body(body.get('name')).id
        ET.SubElement(body, 'inertial',
                      mass=str(model.body_mass[i]),
                      pos=' '.join(map(str, model.body_ipos[i])),
                      quat=' '.join(map(str, model.body_iquat[i])),
                      diaginertia=' '.join(map(str, model.body_inertia[i])))
    placements = {'toaster_main_group_main': (3.65, -.32),
                  'knife_block_main_group_main': (4.1, -.32),
                  'paper_towel_main_group_main': (1.4, -.32),
                  'coffee_machine_main_group_main': (4.65, -.32),
                  'plant_island_group_main': (3.8, -2.65)}
    for name, (x, y) in placements.items():
        i = model.body(name).id
        ids = set()
        for b in range(model.nbody):
            parent = b
            while parent and parent != i:
                parent = int(model.body_parentid[parent])
            if parent == i:
                ids.add(b)
        bottom = math.inf
        for g in range(model.ngeom):
            if int(model.geom_bodyid[g]) not in ids or not model.geom_contype[g]:
                continue
            bounds = model.geom_aabb[g]
            rotation = data.geom_xmat[g].reshape(3, 3)
            center = data.geom_xpos[g] + rotation @ bounds[:3]
            extent = np.abs(rotation) @ bounds[3:]
            bottom = min(bottom, float(center[2] - extent[2]))
        body = next(b for b in root.iter('body') if b.get('name') == name)
        body.set('pos', f'{x} {y} {.92 - bottom + .0001:.12g}')
    for b in list(root.find('worldbody')):
        if b.get('mocap') == 'true':
            root.find('worldbody').remove(b)
        elif b.tag == 'body':
            b.set('childclass', 'kitchen')
    for e in root.iter():
        if e.get('file'):
            e.set('file', 'raw/' + e.get('file'))
        if e.tag == 'geom':
            physical = int(e.get('contype', '1')) or int(e.get('conaffinity', '1'))
            e.set('group', '3' if physical else '2')
            if physical:
                e.set('contype', '1')
                # Robot and free-object collision geoms accept bit 1, while
                # fixed and articulated kitchen geoms must not collide with
                # each other at their authored closed poses.
                e.set('conaffinity', '0')
                e.set('rgba', '.45 .45 .45 1')
    for tag in ('compiler', 'option', 'size'):
        e = root.find(tag)
        if e is not None:
            root.remove(e)
    write_xml(root, directory / 'environment.xml')
    # Fixed fixtures belong to the world, not hundreds of fixed physics bodies.
    # Transform authored geom frames (not compiled mesh inertia frames) once.
    static = copy.deepcopy(root)
    world = static.find('worldbody')
    world.clear()
    def transform(element):
        position = np.array(list(map(float, element.get('pos', '0 0 0').split())))
        quaternion = np.array(list(map(float, element.get('quat', '1 0 0 0').split())))
        if element.get('euler'):
            mujoco.mju_euler2Quat(quaternion, np.array(list(map(float, element.get('euler').split()))), 'xyz')
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, quaternion / np.linalg.norm(quaternion))
        return rotation.reshape(3, 3), position
    def flatten(element, parent_rotation, parent_position, target, retain_joints=False):
        rotation, position = transform(element)
        rotation, position = parent_rotation @ rotation, parent_position + parent_rotation @ position
        active = element.tag == 'body' and any(j.get('name') in INTERACTIVE_JOINTS
                                               for j in element.findall('joint'))
        if retain_joints and active:
            body = copy.deepcopy(element)
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, rotation.ravel())
            body.set('pos', ' '.join(format(v, '.12g') for v in position))
            body.set('quat', ' '.join(format(v, '.12g') for v in quaternion))
            body.attrib.pop('euler', None)
            body.set('childclass', 'kitchen')
            for child in body.iter('body'):
                for joint in list(child.findall('joint')):
                    if joint.get('name') not in INTERACTIVE_JOINTS:
                        child.remove(joint)
            target.append(body)
            return
        if element.tag == 'geom':
            geom = copy.deepcopy(element)
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, rotation.ravel())
            geom.set('pos', ' '.join(format(v, '.12g') for v in position))
            geom.set('quat', ' '.join(format(v, '.12g') for v in quaternion))
            geom.attrib.pop('euler', None)
            geom.set('class', 'kitchen')
            target.append(geom)
        elif element.tag == 'body':
            for child in element:
                if child.tag in ('body', 'geom'):
                    flatten(child, rotation, position, target, retain_joints)
    for element in root.find('worldbody'):
        if element.tag == 'light':
            world.append(copy.deepcopy(element))
        else:
            flatten(element, np.eye(3), np.zeros(3), world)
    write_xml(static, directory / 'environment_static.xml')
    interactive = copy.deepcopy(root)
    world = interactive.find('worldbody')
    world.clear()
    for element in root.find('worldbody'):
        if element.tag == 'light':
            world.append(copy.deepcopy(element))
        else:
            flatten(element, np.eye(3), np.zeros(3), world, retain_joints=True)
    write_xml(interactive, directory / 'environment_interactive.xml')

    objects = ET.Element('mujoco', model='kitchen_objects')
    world = ET.SubElement(objects, 'worldbody')
    body = ET.SubElement(world, 'body', name='grasp_object_box', pos='1.95 -.45 .95')
    ET.SubElement(body, 'freejoint', name='grasp_object_box_joint')
    ET.SubElement(body, 'inertial', pos='0 0 0', mass='.05',
                  diaginertia='.000021666667 .000021666667 .000013333333')
    ET.SubElement(body, 'geom', name='grasp_object_box_collision', type='box',
                  size='.02 .02 .03', group='1', rgba='.8 .25 .15 1',
                  contype='1', conaffinity='1', friction='1 .005 .0001',
                  solref='.002 1', solimp='.99 .999 .0001')
    contact = ET.SubElement(objects, 'contact')
    for side in ('left', 'right'):
        for finger in ('left', 'right'):
            for index in range(4):
                ET.SubElement(contact, 'pair', geom1='grasp_object_box_collision',
                              geom2=f'{side}_fr3v2_1_{finger}finger_pad_{index}',
                              condim='4', friction='1 1 .005 .0001 .0001',
                              solref='.002 1', solimp='.99 .999 .0001')
    write_xml(objects, directory / 'task_objects.xml')
    posture = yaml.safe_load((robot_dir.parent / 'mfr3duo_nav/config/navigation_posture.yaml').read_text())
    config = {'spawn': [2.45, -1.45, math.pi / 2], 'initial_posture': posture,
              'localization': 'simulation_ground_truth',
              'grasp_objects': 'box=grasp_object_box=grasp_object_box_collision',
              'waypoints': {'counter': [2.45, -1.45, math.pi / 2],
                            'aisle': [2.7, -1.45, math.pi / 2]},
              'place_pose': [1.95, -.45, .95],
              'interactive_joints': INTERACTIVE_JOINTS}
    (directory / 'scene.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    profile = yaml.safe_load((robot_dir / 'config/grasp.yaml').read_text())
    profile['observation']['objects']['box']['dimensions'] = [.04, .04, .06]
    profile['fixture'].update(initial_keyframe='kitchen_home', place_clearance=.08,
                              table_position=[3.15, -.325, .905],
                              table_dimensions=[3.8, .65, .03])
    profile['fixture']['pregrasp_joint_constraints'] = {'joint1': [0., .6], 'joint3': [0., .7]}
    (directory / 'grasp.yaml').write_text(yaml.safe_dump(profile, sort_keys=False))
    (directory / 'navigation_posture.yaml').write_text(yaml.safe_dump(posture, sort_keys=False))

    # Derive all static obstacles from the same adapted model used in physics.
    from mfr3duo_scenes.compose import compose_scene
    compiled, _ = compose_scene(directory, robot_dir.parent / 'mfr3duo_description',
                                environment_file='environment.xml', interactive=True,
                                output_dir=Path(tempfile.mkdtemp(prefix='kitchen-prepare-')))
    full = mujoco.MjModel.from_xml_path(str(compiled))
    state = mujoco.MjData(full)
    mujoco.mj_resetDataKeyframe(full, state, full.key('kitchen_home').id)
    mujoco.mj_forward(full, state)
    top_bodies = {b.get('name') for b in root.find('worldbody') if b.tag == 'body'}
    grouped = {}
    articulated = {name: [] for name in config['interactive_joints']}
    stationary = {}
    collision_bounds = []
    for i in range(full.ngeom):
        if full.geom_group[i] != 3 or not full.geom_contype[i]:
            continue
        b = int(full.geom_bodyid[i])
        while int(full.body_parentid[b]) > 0:
            b = int(full.body_parentid[b])
        name = mujoco.mj_id2name(full, mujoco.mjtObj.mjOBJ_BODY, b)
        if name not in top_bodies:
            continue
        a = full.geom_aabb[i]
        rotation = state.geom_xmat[i].reshape(3, 3)
        center = state.geom_xpos[i] + rotation @ a[:3]
        extent = np.abs(rotation) @ a[3:]
        lo, hi = center - extent, center + extent
        if lo[2] > 4:
            continue  # exporter mass helpers at z=10, not furniture
        grouped.setdefault(name, []).append((lo, hi))
        ancestor = int(full.geom_bodyid[i])
        joint_name = None
        while ancestor:
            for joint in range(int(full.body_jntadr[ancestor]),
                               int(full.body_jntadr[ancestor] + full.body_jntnum[ancestor])):
                candidate = mujoco.mj_id2name(full, mujoco.mjtObj.mjOBJ_JOINT, joint)
                if candidate in articulated:
                    joint_name = candidate
                    break
            if joint_name:
                break
            ancestor = int(full.body_parentid[ancestor])
        if joint_name:
            articulated[joint_name].append((lo, hi))
        else:
            stationary.setdefault(name, []).append((lo, hi))
        if 'floor_' not in name and hi[2] > .04 and lo[2] < 1.5:
            collision_bounds.append((lo, hi))
    planning = []
    for name, bounds in grouped.items():
        if name == 'counter_main_main_group_main':
            continue  # exact support geometry is inserted by the grasp profile
        lo = np.min([v[0] for v in bounds], axis=0)
        hi = np.max([v[1] for v in bounds], axis=0)
        if 'floor_' in name:
            hi[2] -= .004  # planning clearance for compliant wheel-ground contact
        planning.append({'id': 'kitchen/' + name, 'position': ((lo + hi) / 2).tolist(),
                         'dimensions': (hi - lo).tolist()})
    (directory / 'planning_scene.yaml').write_text(yaml.safe_dump({'frame': 'simulation_world',
                                                                 'boxes': planning}, sort_keys=False))
    interactive_planning = []
    for name, bounds in stationary.items():
        if name == 'counter_main_main_group_main':
            continue
        lo = np.min([v[0] for v in bounds], axis=0)
        hi = np.max([v[1] for v in bounds], axis=0)
        if 'floor_' in name:
            hi[2] -= .004
        interactive_planning.append({'id': 'kitchen/' + name,
                                     'position': ((lo + hi) / 2).tolist(),
                                     'dimensions': (hi - lo).tolist()})
    for name, bounds in articulated.items():
        joint = full.joint(name).id
        lo = np.min([v[0] for v in bounds], axis=0)
        hi = np.max([v[1] for v in bounds], axis=0)
        interactive_planning.append({'id': 'kitchen/joint/' + name,
                                     'position': ((lo + hi) / 2).tolist(),
                                     'dimensions': (hi - lo).tolist(),
                                     'joint': name,
                                     'joint_type': 'slide' if full.jnt_type[joint] == mujoco.mjtJoint.mjJNT_SLIDE else 'hinge',
                                     'axis': state.xaxis[joint].tolist(),
                                     'pivot': state.xanchor[joint].tolist()})
    (directory / 'planning_scene_interactive.yaml').write_text(yaml.safe_dump(
        {'frame': 'simulation_world', 'boxes': interactive_planning}, sort_keys=False))
    map_dir = directory / 'maps'
    map_dir.mkdir(exist_ok=True)
    resolution, origin, width, height = .025, [-1.2, -5.2, 0.], 270, 216
    pixels = bytearray()
    for row in range(height):
        y = origin[1] + (height - row - .5) * resolution
        for col in range(width):
            x = origin[0] + (col + .5) * resolution
            outside = x < -1. or x > 5.3 or y < -5. or y > 0.
            occupied = outside or any(lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]
                                      for lo, hi in collision_bounds)
            pixels.append(0 if occupied else 254)
    (map_dir / 'kitchen.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    (map_dir / 'kitchen.yaml').write_text(yaml.safe_dump(
        dict(image='kitchen.pgm', resolution=resolution, origin=origin,
             negate=0, occupied_thresh=.65, free_thresh=.196, mode='trinary')))
    nav = yaml.safe_load((robot_dir.parent / 'mfr3duo_nav/config/nav2.yaml').read_text())
    nav['amcl']['ros__parameters']['initial_pose'].update(x=2.45, y=-1.45, yaw=math.pi / 2)
    nav['controller_server']['ros__parameters']['FollowPath']['xy_goal_tolerance'] = .02
    nav['controller_server']['ros__parameters']['FollowPath'].update(sim_time=.8, linear_granularity=.025, vx_samples=15, vy_samples=15, vtheta_samples=11)
    nav['controller_server']['ros__parameters']['FollowPath']['PathDist.scale'] = 24.
    nav['controller_server']['ros__parameters']['FollowPath']['GoalDist.scale'] = 32.
    nav['controller_server']['ros__parameters']['FollowPath']['ObstacleFootprint.scale'] = .002
    nav['controller_server']['ros__parameters']['FollowPath']['Twirling.scale'] = .1
    nav['controller_server']['ros__parameters']['general_goal_checker'].update(xy_goal_tolerance=.02, yaw_goal_tolerance=.025)
    for costmap in ('local_costmap', 'global_costmap'):
        p = nav[costmap][costmap]['ros__parameters']
        # Include both arms and every collision geom, without empty AABB corners.
        p['footprint'] = repr(collision_footprint(full, state))
        p['footprint_padding'] = .025
        p['resolution'] = .01 if costmap == 'local_costmap' else .025
        p['inflation_layer']['inflation_radius'] = .45
        p['inflation_layer']['cost_scaling_factor'] = 8.
    (directory / 'nav2.yaml').write_text(yaml.safe_dump(nav, sort_keys=False))
    print('PREPARED', len(planning), 'planning fixtures;', len(collision_bounds), 'physical map proxies')
    import shutil
    shutil.rmtree(compiled.parent)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('robot_dir', type=Path)
    args = parser.parse_args()
    prepare(args.directory.resolve(), args.robot_dir.resolve())
