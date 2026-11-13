"""Build a traced MJCF with separate source visuals and collision envelopes.

No upstream file is modified. Mesh collision uses its convex envelope, as does
MuJoCo's mesh contact solver. Explicit URDF mass/inertia is never recomputed.
"""
from pathlib import Path
import hashlib
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = ROOT / 'upstream/fudan_rl_wheel_leg'
SOURCE = UPSTREAM / 'plane/resources/robots/infantry_V4/urdf/infantry_V4.urdf'
OUTPUT = ROOT / 'assets/robots/infantry_v4_standard_simplified'
COLOR_SOURCE = UPSTREAM / 'mujoco/assert_now/infantry_binglian_yuntai/infantry_V2/meshes/mjmodel.xml'
ORDER = ['lf0_Joint', 'lf1_Joint', 'l_wheel_Joint', 'rf0_Joint', 'rf1_Joint', 'r_wheel_Joint']


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def pose(origin):
    if origin is None:
        return {'pos': '0 0 0', 'quat': '1 0 0 0'}
    xyzw = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_quat()
    return {'pos': origin.get('xyz', '0 0 0'), 'quat': ' '.join(map(str, xyzw[[3, 0, 1, 2]]))}


def split_visual_stl(source, stem, max_faces=190000):
    """Preserve each binary STL triangle byte-for-byte; split for decoder limit.

    No simplification, recentering, hull replacement, or geometry scaling.
    """
    with source.open('rb') as stream:
        header = stream.read(80)
        count = struct.unpack('<I', stream.read(4))[0]
        if source.stat().st_size != 84 + 50*count:
            raise ValueError(f'Expected binary STL with exact record length: {source}')
        paths = []
        triangle_hash = hashlib.sha256()
        reconstructed_hash = hashlib.sha256()
        for index, offset in enumerate(range(0, count, max_faces)):
            n = min(count-offset, max_faces)
            records = stream.read(50*n)
            triangle_hash.update(records)
            target = OUTPUT / f'{stem}_visual_{index:02d}.stl'
            target.write_bytes(header + struct.pack('<I', n) + records)
            with target.open('rb') as derived:
                derived.seek(84)
                reconstructed_hash.update(derived.read())
            paths.append(dict(file=target.name, faces=n, sha256=digest(target)))
        if triangle_hash.digest() != reconstructed_hash.digest():
            raise ValueError(f'Visual triangle preservation failed: {source}')
        return paths, count, triangle_hash.hexdigest()


def build():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    robot = ET.parse(SOURCE).getroot()
    color_source = ET.parse(COLOR_SOURCE).getroot()
    # User-selected appearance only. Do not import parallel-model physics.
    palette = {b.get('name'):b.find('geom').get('rgba') for b in color_source.findall('.//body') if b.find('geom') is not None}
    palette['base_link'] = palette['base_Link_del']
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = robot.findall('joint')
    assert len(joints) == 6 and all(j.get('type') == 'revolute' for j in joints)
    root = ET.Element('mujoco', model='fudan_infantry_v4_standard_simplified')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false', autolimits='true')
    ET.SubElement(root, 'option', timestep='0.002', gravity='0 0 -9.81', integrator='implicitfast')
    defaults = ET.SubElement(root, 'default')
    ET.SubElement(defaults, 'geom', solref='0.005 1')
    assets = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', name='visual_key_light', pos='1 -1 2', dir='-0.4 0.3 -1',
                  directional='true', diffuse='0.8 0.8 0.8', ambient='0.2 0.2 0.2')
    # Explicit project contact settings, not measurements of the real robot.
    ET.SubElement(world, 'geom', name='floor', type='plane', size='5 5 0.1',
                  contype='2', conaffinity='1', friction='1 0.005 0.0001', rgba='0.7 0.7 0.7 1')
    children = {}
    for joint in joints:
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    mesh_manifest = []
    visual_manifest = []

    def add_link(name, parent, joint=None):
        link = links[name]
        body = ET.SubElement(parent, 'body', name=name, **(pose(joint.find('origin')) if joint is not None else {'pos': '0 0 0.4'}))
        inertial = link.find('inertial')
        inertia = inertial.find('inertia')
        # MuJoCo fullinertia is expressed in the body's frame; rotate URDF tensor.
        tensor = np.array([[float(inertia.get('ixx')), float(inertia.get('ixy')), float(inertia.get('ixz'))],
                           [float(inertia.get('ixy')), float(inertia.get('iyy')), float(inertia.get('iyz'))],
                           [float(inertia.get('ixz')), float(inertia.get('iyz')), float(inertia.get('izz'))]])
        origin = inertial.find('origin')
        rot = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        tensor = rot @ tensor @ rot.T
        eigenvalues = np.linalg.eigvalsh(tensor)
        if eigenvalues.min() <= 0 or eigenvalues[0] + eigenvalues[1] < eigenvalues[2]:
            raise ValueError(f'Nonphysical inertia in {name}: {eigenvalues}')
        ET.SubElement(body, 'inertial', pos=origin.get('xyz', '0 0 0'), mass=inertial.find('mass').get('value'),
                      fullinertia=' '.join(map(str, [tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]])))
        if joint is None:
            ET.SubElement(body, 'freejoint', name='root')
        else:
            limit = joint.find('limit')
            attrs = dict(name=joint.get('name'), type='hinge', axis=joint.find('axis').get('xyz'),
                         actuatorfrcrange=f"-{limit.get('effort')} {limit.get('effort')}")
            # Wheels have no URDF angle bounds: explicitly continuous in MJCF.
            if 'lower' in limit.attrib and 'upper' in limit.attrib:
                attrs.update(limited='true', range=f"{limit.get('lower')} {limit.get('upper')}")
            else:
                attrs['limited'] = 'false'
            ET.SubElement(body, 'joint', **attrs)
        for index, collision in enumerate(link.findall('collision')):
            geometry = collision.find('geometry')
            mesh = geometry.find('mesh')
            if mesh is None:
                raise ValueError('Expected upstream mesh geometry; refuse implicit geometry replacement')
            source_mesh = (SOURCE.parent / mesh.get('filename')).resolve()
            mesh_name = f'{name}_{index}'
            target = OUTPUT / f'{mesh_name}_hull.stl'
            loaded = trimesh.load_mesh(source_mesh, process=False)
            if 'scale' in mesh.attrib:
                loaded.apply_scale(np.fromstring(mesh.get('scale'), sep=' '))
            hull = loaded.convex_hull
            hull.export(target)
            assert np.allclose(loaded.bounds, hull.bounds, atol=1e-9, rtol=0)
            mesh_manifest.append(dict(source=str(source_mesh.relative_to(ROOT)), source_sha256=digest(source_mesh),
                                      derived=target.name, sha256=digest(target), source_faces=len(loaded.faces),
                                      hull_faces=len(hull.faces), bounds_m=hull.bounds.tolist()))
            ET.SubElement(assets, 'mesh', name=mesh_name, file=target.name)
            # Robot-vs-robot contacts disabled, matching upstream training config.
            ET.SubElement(body, 'geom', name=f'{name}_collision_{index}', type='mesh', mesh=mesh_name,
                          contype='1', conaffinity='2', group='3', friction='1 0.005 0.0001',
                          rgba='0.25 0.45 0.65 1', **pose(collision.find('origin')))
        for index, visual in enumerate(link.findall('visual')):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Expected source mesh visual geometry')
            source_mesh = (SOURCE.parent / mesh.get('filename')).resolve()
            color = visual.find('material/color')
            if color is None:
                raise ValueError('Visual material must be resolved from source; refuse invented color')
            rgba = palette[name]
            stem = f'{name}_{index}'
            pieces, faces, triangle_sha256 = split_visual_stl(source_mesh, stem)
            for piece_index, piece in enumerate(pieces):
                mesh_name = f'{stem}_visual_{piece_index}'
                attributes = dict(name=mesh_name, file=piece['file'])
                if 'scale' in mesh.attrib:
                    attributes['scale'] = mesh.get('scale')
                ET.SubElement(assets, 'mesh', **attributes)
                ET.SubElement(body, 'geom', name=mesh_name, type='mesh', mesh=mesh_name,
                    group='2', contype='0', conaffinity='0', mass='0', rgba=rgba, **pose(visual.find('origin')))
            visual_manifest.append(dict(source=str(source_mesh.relative_to(ROOT)), source_sha256=digest(source_mesh),
                link=name, rgba=rgba, original_urdf_rgba=color.get('rgba'), source_faces=faces, output_faces=sum(p['faces'] for p in pieces),
                triangle_records_sha256=triangle_sha256, exact_triangle_preservation=True, chunks=pieces))
        for child in children.get(name, []):
            add_link(child.find('child').get('link'), body, child)

    add_link('base_link', world)
    actuators = ET.SubElement(root, 'actuator')
    for name in ORDER:
        joint = next(j for j in joints if j.get('name') == name)
        effort = joint.find('limit').get('effort')
        ET.SubElement(actuators, 'motor', name=f'motor_{name}', joint=name, gear='1',
                      ctrllimited='true', ctrlrange=f'-{effort} {effort}')
    ET.indent(root)
    ET.ElementTree(root).write(OUTPUT / 'robot.xml', encoding='utf-8', xml_declaration=True)
    manifest = dict(source=str(SOURCE.relative_to(ROOT)), source_sha256=digest(SOURCE),
                    upstream_commit='8204e853dfd2ed06d85a322e1a998c3d20a3be2c',
                    representation='upstream six-joint serial simplification; not physical parallel mechanism',
                    collision='convex envelope of each upstream collision mesh; no inertia recomputation',
                    visual='Exact source STL triangles; user-selected upstream MJCF white-body/dark-gray-wheel palette',
                    color_source=str(COLOR_SOURCE.relative_to(ROOT)), color_source_sha256=digest(COLOR_SOURCE),
                    color_scope='Appearance only; no parallel-model mass, joint or collision parameters reused',
                    project_settings=dict(physics_dt=0.002, gravity=[0, 0, -9.81], friction=[1, 0.005, 0.0001],
                                          contact_solref=[0.005, 1], self_collision=False),
                    action_order=ORDER, meshes=mesh_manifest, visual_meshes=visual_manifest)
    (OUTPUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(OUTPUT / 'robot.xml')


if __name__ == '__main__':
    build()
