"""Scene contracts against MuJoCo: physical transforms, relocation and map fit."""
from pathlib import Path
import ctypes
import math
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

from mfr3duo_scenes.compose import compose_scene

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'scenes/kitchen'
DESCRIPTION = ROOT.parent / 'mfr3duo_description'


def model(path):
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key('kitchen_home').id)
    mujoco.mj_forward(m, d)
    return m, d


def test_flattening_preserves_visual_and_collision_frames(tmp_path):
    fixed, _ = compose_scene(SCENE, DESCRIPTION, output_dir=tmp_path / 'fixed')
    moving, config = compose_scene(SCENE, DESCRIPTION, interactive=True, output_dir=tmp_path / 'moving')
    fm, fd = model(fixed)
    mm, md = model(moving)
    assert fm.nq == 42 and mm.nq == 44
    assert 0 < mm.nbody - fm.nbody < 20
    environment_names = {g.get('name') for g in ET.parse(SCENE / 'environment.xml').getroot().iter('geom')}
    for i in range(mm.ngeom):
        name = mujoco.mj_id2name(mm, mujoco.mjtObj.mjOBJ_GEOM, i)
        if not name or name not in environment_names:
            continue
        j = fm.geom(name).id
        np.testing.assert_allclose(fd.geom_xpos[j], md.geom_xpos[i], atol=1e-6)
        np.testing.assert_allclose(fd.geom_xmat[j], md.geom_xmat[i], atol=1e-6)
        assert fm.geom_group[j] == mm.geom_group[i]
        if fm.geom_contype[j]:
            assert fm.geom_contype[j] == 1 and fm.geom_conaffinity[j] == 0
    assert 'bottom_main_group_1_slidejoint' in config['scene_joints']
    assert 'top_main_group_leftdoorhinge' in config['scene_joints']
    assert mm.nu == fm.nu + 2
    assert all(c.dist > -.0005 for c in fd.contact)
    base = fm.joint('base_freejoint').qposadr[0]
    np.testing.assert_allclose(fd.qpos[base:base + 3], [2.45, -1.45, .002])
    assert not np.any(np.isnan(md.qpos))
    equality_rows = fd.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    assert np.max(np.abs(fd.efc_pos[equality_rows])) < 1e-6


def test_installed_resources_can_relocate(tmp_path):
    scene = tmp_path / 'isolated/scenes/share/mfr3duo_scenes/scenes/kitchen'
    description = tmp_path / 'isolated/description/share/mfr3duo_description'
    shutil.copytree(SCENE, scene, ignore=shutil.ignore_patterns('.validation'))
    shutil.copytree(DESCRIPTION / 'mjcf', description / 'mjcf')
    path, _ = compose_scene(scene, description, output_dir=tmp_path / 'generated')
    fm, _ = model(path)
    assert fm.ncam == 14
    text = path.read_text()
    assert str(SCENE) not in text and str(DESCRIPTION) not in text


def test_navigation_footprint_fits_spawn_and_waypoint():
    config = yaml.safe_load((SCENE / 'scene.yaml').read_text())
    metadata = yaml.safe_load((SCENE / 'maps/kitchen.yaml').read_text())
    _, dimensions, _, pixels = (SCENE / 'maps/kitchen.pgm').read_bytes().split(b'\n', 3)
    width, height = map(int, dimensions.split())
    ox, oy, _ = metadata['origin']
    resolution = metadata['resolution']
    for x, y, yaw in (config['spawn'], *config['waypoints'].values()):
        # Include footprint padding and interior, not only its corners.
        for px in np.linspace(-.435, .745, 30):
            for py in np.linspace(-1.095, 1.045, 45):
                wx = x + math.cos(yaw) * px - math.sin(yaw) * py
                wy = y + math.sin(yaw) * px + math.cos(yaw) * py
                col, row = int((wx - ox) / resolution), height - 1 - int((wy - oy) / resolution)
                assert 0 <= col < width and 0 <= row < height
                assert pixels[row * width + col] == 254, (wx, wy)


def test_navigation_hull_contains_all_robot_collision_geometry(tmp_path):
    path, _ = compose_scene(SCENE, DESCRIPTION, output_dir=tmp_path)
    fm, fd = model(path)
    nav = yaml.safe_load((SCENE / 'nav2.yaml').read_text())
    footprint = np.array(yaml.safe_load(nav['local_costmap']['local_costmap']['ros__parameters']['footprint']))
    base = int(fm.joint('base_freejoint').bodyid[0])
    rotation = fd.xmat[base].reshape(3, 3)
    for i in range(fm.ngeom):
        ancestor = int(fm.geom_bodyid[i])
        while ancestor and ancestor != base:
            ancestor = int(fm.body_parentid[ancestor])
        if ancestor != base or not fm.geom_contype[i]:
            continue
        for signs in ((x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)):
            world = fd.geom_xpos[i] + fd.geom_xmat[i].reshape(3, 3) @ (
                fm.geom_aabb[i, :3] + fm.geom_aabb[i, 3:] * signs)
            point = (rotation.T @ (world - fd.xpos[base]))[:2]
            for a, b in zip(footprint, np.roll(footprint, -1, axis=0)):
                delta, offset = b - a, point - a
                assert delta[0] * offset[1] - delta[1] * offset[0] >= -1e-6


def test_native_3_12_compiles_both_profiles(tmp_path):
    native = ROOT.parents[1] / 'deps/romujoco-install/lib/libmujoco.so.3.12.0'
    library = ctypes.CDLL(str(native))
    library.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
    library.mj_loadXML.restype = ctypes.c_void_p
    library.mj_deleteModel.argtypes = [ctypes.c_void_p]
    for interactive in (False, True):
        path, _ = compose_scene(SCENE, DESCRIPTION, interactive=interactive,
                                output_dir=tmp_path / str(interactive))
        error = ctypes.create_string_buffer(4096)
        compiled = library.mj_loadXML(str(path).encode(), None, error, len(error))
        assert compiled, error.value.decode()
        library.mj_deleteModel(compiled)


def test_counter_grasp_clears_the_physical_hand_geometry(tmp_path):
    path, _ = compose_scene(SCENE, DESCRIPTION, output_dir=tmp_path)
    fm, fd = model(path)
    profile = yaml.safe_load((SCENE / 'grasp.yaml').read_text())
    box = fm.geom('grasp_object_box_collision').id
    assert fm.geom_contype[box] == fm.geom_conaffinity[box] == 1
    np.testing.assert_allclose(fm.geom_size[box] * 2,
                               profile['observation']['objects']['box']['dimensions'])
    table = profile['fixture']
    top = table['table_position'][2] + table['table_dimensions'][2] / 2
    tcp = fm.body('left_fr3v2_1_hand_tcp').id
    rotation = fd.xmat[tcp].reshape(3, 3)
    # Downward grasp: maximum local TCP z is the lowest hand point in world z.
    protrusion = 0.
    for geom in range(fm.ngeom):
        body = fm.body(int(fm.geom_bodyid[geom])).name
        if not body.startswith('left_fr3v2_1_') or fm.geom_contype[geom] != 1:
            continue
        if not body.endswith('_hand') and 'finger' not in body:
            continue
        relative_rotation = rotation.T @ fd.geom_xmat[geom].reshape(3, 3)
        center = rotation.T @ (fd.geom_xpos[geom] - fd.xpos[tcp]) + relative_rotation @ fm.geom_aabb[geom, :3]
        extent = np.abs(relative_rotation) @ fm.geom_aabb[geom, 3:]
        protrusion = max(protrusion, center[2] + extent[2])
    robot_parameters = yaml.safe_load((ROOT.parent / 'mfr3duo_robot/config/robot.yaml').read_text())
    offset = robot_parameters['/**']['ros__parameters']['robot']['grasp_tcp_offset']
    assert fd.geom_xpos[box, 2] + offset - protrusion - top >= .005


def test_articulated_planning_boxes_cover_measured_fixture_geometry(tmp_path):
    path, _ = compose_scene(SCENE, DESCRIPTION, interactive=True, output_dir=tmp_path)
    fm, fd = model(path)
    entries = yaml.safe_load((SCENE / 'planning_scene_interactive.yaml').read_text())['boxes']
    for entry in entries:
        if 'joint' not in entry:
            continue
        joint = fm.joint(entry['joint']).id
        axis = np.array(entry['axis'])
        pivot = np.array(entry['pivot'])
        center = np.array(entry['position'])
        half = np.array(entry['dimensions']) / 2
        body = int(fm.jnt_bodyid[joint])
        for value in np.linspace(*fm.jnt_range[joint], 5):
            fd.qpos[fm.jnt_qposadr[joint]] = value
            mujoco.mj_forward(fm, fd)
            rotation = np.eye(3)
            if entry['joint_type'] == 'hinge':
                quat = np.empty(4)
                mat = np.empty(9)
                mujoco.mju_axisAngle2Quat(quat, axis, value)
                mujoco.mju_quat2Mat(mat, quat)
                rotation = mat.reshape(3, 3)
                position = pivot + rotation @ (center - pivot)
            else:
                position = center + axis * value
            for i in range(fm.ngeom):
                ancestor = int(fm.geom_bodyid[i])
                while ancestor and ancestor != body:
                    ancestor = int(fm.body_parentid[ancestor])
                if ancestor != body or fm.geom_group[i] != 3 or not fm.geom_contype[i]:
                    continue
                geom_rotation = fd.geom_xmat[i].reshape(3, 3)
                for signs in ((x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)):
                    point = fd.geom_xpos[i] + geom_rotation @ (
                        fm.geom_aabb[i, :3] + fm.geom_aabb[i, 3:] * signs)
                    local = rotation.T @ (point - position)
                    assert np.all(np.abs(local) <= half + 1e-6), (entry['joint'], value, local, half)
