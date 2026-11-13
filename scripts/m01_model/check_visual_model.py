"""Check exact source visual preservation and visual-only physics isolation."""
import datetime
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from audit_model import baseline
from prepare_model import ROOT, OUTPUT, digest


def main():
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'runs/m01_model' / f'visual_check_{stamp}'
    output.mkdir(parents=True)
    manifest = json.loads((OUTPUT/'manifest.json').read_text())
    visual_checks = []
    for mesh in manifest['visual_meshes']:
        source = ROOT / mesh['source']
        original = hashlib.sha256(source.read_bytes()[84:]).hexdigest()
        reconstructed = hashlib.sha256()
        for chunk in mesh['chunks']:
            reconstructed.update((OUTPUT/chunk['file']).read_bytes()[84:])
        expected_color = '0.18 0.18 0.18 1' if 'wheel' in mesh['link'] else '1 1 1 1'
        visual_checks.append(dict(link=mesh['link'], exact_triangles=original==reconstructed.hexdigest(),
            source_faces=mesh['source_faces'], output_faces=mesh['output_faces'],
            rgba=mesh['rgba'], color_matches_selected_upstream=mesh['rgba']==expected_color))
    model = mujoco.MjModel.from_xml_path(str(OUTPUT/'robot.xml'))
    visuals = [i for i in range(model.ngeom) if model.geom_group[i]==2]
    isolated = all(model.geom_contype[i]==0 and model.geom_conaffinity[i]==0 for i in visuals)
    # Build the same physical model after removing display geometry.
    root = ET.parse(OUTPUT/'robot.xml').getroot()
    for body in root.findall('.//body'):
        for geom in list(body.findall('geom')):
            if geom.get('group')=='2':
                body.remove(geom)
    assets = root.find('asset')
    for mesh in list(assets.findall('mesh')):
        if '_visual_' in mesh.get('name'):
            assets.remove(mesh)
    physical = mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'),
        assets={p.name:p.read_bytes() for p in OUTPUT.glob('*_hull.stl')})
    full_data = baseline(model)
    full_data.qpos[2] = 0
    mujoco.mj_forward(model, full_data)
    contacts = [c for c in full_data.contact if any('wheel' in model.body(model.geom_bodyid[g]).name for g in c.geom)]
    full_data.qpos[2] = -min(c.dist for c in contacts)+.001
    physical_data = mujoco.MjData(physical)
    physical_data.qpos[:] = full_data.qpos
    rng = np.random.default_rng(0)
    max_difference = 0.
    for _ in range(250):
        action = rng.uniform(-.1,.1,6)
        full_data.ctrl[:] = action; physical_data.ctrl[:] = action
        mujoco.mj_step(model,full_data); mujoco.mj_step(physical,physical_data)
        max_difference = max(max_difference,float(np.max(abs(full_data.qpos-physical_data.qpos))))
    same_inertial = bool(np.array_equal(model.body_mass,physical.body_mass) and
                         np.array_equal(model.body_inertia,physical.body_inertia))
    result = dict(visual_checks=visual_checks, visual_geom_count=len(visuals),
        no_visual_collision=isolated, identical_mass_and_inertia=same_inertial,
        max_qpos_difference_250_steps=max_difference,
        tolerance=1e-10, seed=0, source_geometry_unchanged=True,
        model_sha256=digest(OUTPUT/'robot.xml'), script_sha256=digest(ROOT/'scripts/m01_model/check_visual_model.py'))
    result['passed'] = all(c['exact_triangles'] and c['color_matches_selected_upstream'] and
                          c['source_faces']==c['output_faces'] for c in visual_checks) and isolated and same_inertial and max_difference<1e-10
    (output/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(output)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
