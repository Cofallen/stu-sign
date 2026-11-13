"""Derive no-spring and compressible spring/damper XMLs without rigid CAD rods.

The spring uses a spatial tendon: no added rigid loop, DOF, or length lock.
Its dynamics are an explicitly uncalibrated equivalent model, not gas thermodynamics.
"""
import argparse
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from prepare_model import ROOT, OUTPUT, UPSTREAM, COLOR_SOURCE, digest


def remove_rigid_spring(name):
    source = UPSTREAM / f'plane/resources/robots/infantry_V4/meshes/{name}.STL'
    data = source.read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    mesh = trimesh.load_mesh(source, process=False)
    assert len(mesh.faces) == count
    # Merge identical vertex positions only. Preserve face indexing and raw records.
    mesh.merge_vertices()
    components = trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(count))
    removed = np.zeros(count, dtype=bool)
    selections = []
    for component in components:
        points = mesh.vertices[mesh.faces[component]].reshape(-1,3)
        bounds = np.array([points.min(0),points.max(0)])
        extent = bounds[1]-bounds[0]
        center = bounds.mean(0)
        # Source-specific CAD identification: coaxial cylinder and rod, not leg bars.
        if extent[0] > .08 and extent[1] < .016 and extent[2] < .02 and abs(abs(center[1])-.04502)<.001 and abs(center[2]+.00855)<.001:
            removed[component] = True
            selections.append(dict(faces=len(component), bounds=bounds.tolist()))
    if len(selections)!=2 or removed.sum()!=862:
        raise ValueError(f'Unexpected spring CAD segmentation for {name}: {selections}')
    records = np.frombuffer(data, dtype=np.dtype('V50'), offset=84, count=count)
    target = OUTPUT / f'{name}_without_rigid_gas_spring.stl'
    target.write_bytes(data[:80]+struct.pack('<I', int((~removed).sum()))+records[~removed].tobytes())
    # Store exact source face selection for reproducibility; no mesh approximation.
    np.save(OUTPUT/f'{name}_removed_spring_face_ids.npy',np.flatnonzero(removed))
    retained = trimesh.load_mesh(target,process=False)
    hull_path = OUTPUT / f'{name}_without_rigid_gas_spring_hull.stl'
    retained.convex_hull.export(hull_path)
    return dict(link=name,source=str(source.relative_to(ROOT)),source_sha256=digest(source),
        source_faces=count,removed_faces=int(removed.sum()),retained_faces=int((~removed).sum()),
        removed_components=selections,visual=target.name,visual_sha256=digest(target),
        collision=hull_path.name,collision_sha256=digest(hull_path))


def build(stiffness=1000.,damping=10.,free_length=.20):
    if stiffness<0 or damping<0 or free_length<=0:
        raise ValueError('Use nonnegative stiffness/damping and positive free length')
    root = ET.parse(OUTPUT/'robot.xml').getroot()
    mesh_assets=root.find('asset')
    modifications=[]
    for name in ['lf0_Link','rf0_Link']:
        change=remove_rigid_spring(name);modifications.append(change)
        body=root.find(f'.//body[@name="{name}"]')
        visual=body.find(f'geom[@name="{name}_0_visual_0"]')
        mesh_assets.find(f'mesh[@name="{visual.get("mesh")}"]').set('file',change['visual'])
        collision=body.find(f'geom[@name="{name}_collision_0"]')
        mesh_assets.find(f'mesh[@name="{collision.get("mesh")}"]').set('file',change['collision'])
    root.set('model','fudan_v4_no_gas_spring')
    root.insert(0,ET.Comment('No gas-spring force, damping, tendon, or rigid CAD cylinder/rod. Source lumped inertias retained.'))
    # XML files are independently loadable with their sibling mesh assets.
    no_spring=OUTPUT/'robot_no_gas_spring.xml'
    ET.indent(root);ET.ElementTree(root).write(no_spring,encoding='utf-8',xml_declaration=True)
    root.set('model','fudan_v4_compressible_gas_spring')
    root.remove(list(root)[0])
    root.insert(0,ET.Comment('Equivalent compression-only spring/damper. UNCALIBRATED demo k[N/m], c[N*s/m], free length[m]. No hard stroke constraint.'))
    source=ET.parse(COLOR_SOURCE).getroot()
    tendon=ET.SubElement(root,'tendon')
    attachment_records=[]
    for side,prefix in [('left','l'),('right','r')]:
        upper=f'{prefix}f0_Link';lower=f'{prefix}f1_Link'
        ids=[]
        for link,level in [(upper,0),(lower,1)]:
            source_name=f'{side}_lf{level}_qitanhuang'
            source_site=source.find(f'.//site[@name="{source_name}"]')
            site_name=f'{side}_gas_spring_{"upper" if level==0 else "lower"}'
            ET.SubElement(root.find(f'.//body[@name="{link}"]'),'site',name=site_name,
                pos=source_site.get('pos'),size='0.003',rgba='0.6 0.6 0.6 1',group='4')
            ids.append(site_name)
            attachment_records.append(dict(site=site_name,link=link,pos=source_site.get('pos'),source_site=source_name))
        element=ET.SubElement(tendon,'spatial',name=f'{side}_gas_spring',width='0.0075',rgba='0.65 0.65 0.65 1',
            stiffness=str(stiffness),damping=str(damping),springlength=f'{free_length} 1',limited='false',frictionloss='0')
        element.append(ET.Comment('EDIT damping for viscous resistance. springlength upper=1m is slack deadband, NOT stroke limit.'))
        for site in ids:ET.SubElement(element,'site',site=site)
    spring_path=OUTPUT/'robot_gas_spring_damped.xml'
    ET.indent(root);ET.ElementTree(root).write(spring_path,encoding='utf-8',xml_declaration=True)
    manifest=dict(parent_xml_sha256=digest(OUTPUT/'robot.xml'),generator_sha256=digest(ROOT/'scripts/m01_model/build_gas_spring_variants.py'),
        variant_files={no_spring.name:digest(no_spring),spring_path.name:digest(spring_path)},
        mesh_changes=modifications,attachment_source=str(COLOR_SOURCE.relative_to(ROOT)),attachments=attachment_records,
        model='Compression-only equivalent linear spatial tendon; no new DOFs or rigid closure',
        parameters=dict(stiffness_n_m=stiffness,damping_n_s_m=damping,free_length_m=free_length,calibrated=False),
        mass_note='URDF provides lumped link inertias only; retained in both variants. No unsupported gas-spring mass subtraction.',
        visualization='MuJoCo spatial tendon: automatic variable-length display, not a detailed telescopic CAD cylinder',
        attachment_note='Upstream gas-spring sites adapted to standard V4 link frames; hardware attachment calibration pending')
    (OUTPUT/'gas_spring_variants.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print(no_spring);print(spring_path)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--stiffness',type=float,default=1000.)
    parser.add_argument('--damping',type=float,default=10.)
    parser.add_argument('--free-length',type=float,default=.20)
    args=parser.parse_args();build(args.stiffness,args.damping,args.free_length)
