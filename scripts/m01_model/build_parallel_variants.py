"""Keep upstream parallel linkage, all 14 hinges, and four closed pin joints."""
import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares

from prepare_model import ROOT, UPSTREAM, COLOR_SOURCE, OUTPUT as SERIAL_ASSETS, digest

OUTPUT=ROOT/'assets/robots/infantry_parallel'
PASSIVE=['rf1_Joint','r21_Link','r22_Joint','r23_Joint','lf1_Joint','l21_Joint','l22_Joint','l23_Joint']
ACTIVE=['lf0_Joint','l20_Joint','l_wheel_Joint','rf0_Joint','r20_Joint','r_wheel_Joint']


def errors(m,d):
    return {m.equality(i).name:float(np.linalg.norm(d.site_xpos[m.eq_obj1id[i]]-d.site_xpos[m.eq_obj2id[i]])) for i in range(m.neq)}


def solve_assembly(root):
    assets={p.name:p.read_bytes() for p in OUTPUT.glob('*.stl')}
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'),assets=assets)
    d=mujoco.MjData(m);indices=[int(m.joint(n).qposadr[0]) for n in PASSIVE]
    def residual(x):
        d.qpos[indices]=x;mujoco.mj_forward(m,d)
        return np.concatenate([d.site_xpos[m.eq_obj1id[i]]-d.site_xpos[m.eq_obj2id[i]] for i in range(m.neq)])
    solution=least_squares(residual,np.zeros(len(indices)),ftol=1e-13,xtol=1e-13,gtol=1e-13,max_nfev=2000)
    if abs(solution.fun).max()>1e-9:raise RuntimeError('Unable to assemble source linkage')
    # Preserve original absolute hinge coordinate q. At default q=ref, body
    # orientation includes R(ref): R(ref)*R(q-ref)=R(q), all hinge axes local Z.
    rebase=[]
    for name,q in zip(PASSIVE,solution.x):
        body=next(b for b in root.findall('.//body') if b.find(f'joint[@name="{name}"]') is not None)
        joint=body.find(f'joint[@name="{name}"]')
        axis=np.fromstring(joint.get('axis'),sep=' ')
        assert np.allclose(axis,[0,0,1]) and np.allclose(np.fromstring(joint.get('pos','0 0 0'),sep=' '),0)
        rotation=np.array([np.cos(q/2),0,0,np.sin(q/2)])
        quaternion=np.zeros(4)
        mujoco.mju_mulQuat(quaternion,np.fromstring(body.get('quat','1 0 0 0'),sep=' '),rotation)
        body.set('quat',' '.join(map(str,quaternion)));joint.set('ref',str(q))
        rebase.append(dict(joint=name,default_angle_rad=float(q)))
    return rebase


def build(stiffness=1000.,damping=10.,free_length=.20):
    if stiffness<0 or damping<0 or free_length<=0:raise ValueError('Invalid spring parameters')
    OUTPUT.mkdir(parents=True,exist_ok=True)
    root=ET.parse(COLOR_SOURCE).getroot();compiler=root.find('compiler')
    compiler.set('inertiafromgeom','false');compiler.set('autolimits','true')
    option=root.find('option');option.set('timestep','0.001');option.set('integrator','implicitfast')
    world=root.find('worldbody')
    for geom in list(world.findall('geom')):
        if geom.get('name')!='floor':world.remove(geom)
    floor=world.find('geom');floor.attrib.pop('material',None);floor.set('rgba','0.7 0.7 0.7 1')
    ET.SubElement(world,'light',pos='1 -1 2',dir='-0.4 0.3 -1',directional='true',diffuse='0.8 0.8 0.8',ambient='0.2 0.2 0.2')
    base=world.find('body');base.set('pos','0 0 0.4')
    asset=root.find('asset');provenance=[]
    for item in list(asset):
        if item.tag=='hfield':asset.remove(item)
        elif item.tag=='mesh':
            source=(COLOR_SOURCE.parent/item.get('file')).resolve()
            dest=OUTPUT/(item.get('name')+'.stl');shutil.copyfile(source,dest);item.set('file',dest.name)
            provenance.append(dict(mesh=item.get('name'),source=str(source.relative_to(ROOT)),sha256=digest(source)))
    for body in root.findall('.//body'):
        for i,geom in enumerate(list(body.findall('geom'))):
            geom.set('name',f'{body.get("name")}_collision_{i}');geom.set('group','3')
            visual=ET.fromstring(ET.tostring(geom));visual.set('name',f'{body.get("name")}_visual_{i}')
            visual.set('group','2');visual.set('contype','0');visual.set('conaffinity','0');visual.set('mass','0')
            body.append(visual)
        for site in body.findall('site'):site.set('group','4')
    # Keep the full body exterior already approved by user. Appearance only;
    # parallel source mass, inertia, collision, and all link meshes remain intact.
    old=base.find('geom[@group="2"]');base.remove(old)
    serial=ET.parse(SERIAL_ASSETS/'robot.xml').getroot()
    visuals=serial.findall('worldbody/body/geom[@group="2"]')
    for visual in visuals:
        node=ET.fromstring(ET.tostring(visual));node.set('name','body_exterior_'+node.get('name'));base.append(node)
        mesh=serial.find(f'asset/mesh[@name="{node.get("mesh")}"]')
        mesh=ET.fromstring(ET.tostring(mesh));shutil.copyfile(SERIAL_ASSETS/mesh.get('file'),OUTPUT/mesh.get('file'));asset.append(mesh)
        provenance.append(dict(mesh=mesh.get('name'),source=str((SERIAL_ASSETS/mesh.get('file')).relative_to(ROOT)),
            sha256=digest(SERIAL_ASSETS/mesh.get('file')),scope='Approved body exterior only; no physics imported'))
    # All four connect constraints remain. Rear marker shifts along pin axis,
    # accounting for plate thickness; bodies, hinge anchors, and axes unchanged.
    marker_changes=[]
    for site in root.findall('.//site'):
        if 'rear_site' in site.get('name',''):
            position=np.fromstring(site.get('pos'),sep=' ');old=position.copy();position[2]-=.01
            site.set('pos',' '.join(map(str,position)))
            marker_changes.append(dict(site=site.get('name'),before=old.tolist(),after=position.tolist(),
                reason='Same pin-axis point, opposite plate is offset 10mm along local Z'))
    # The physical inputs are two leg motors plus one wheel motor per side.
    for actuator in list(root.find('actuator')):
        if actuator.get('joint') not in ACTIVE:root.find('actuator').remove(actuator)
        else:
            limit=5 if 'wheel' in actuator.get('joint') else 30
            actuator.set('ctrllimited','true');actuator.set('ctrlrange',f'-{limit} {limit}')
    root.remove(root.find('tendon'))
    home=solve_assembly(root)
    manifest=dict(upstream_commit='8204e853dfd2ed06d85a322e1a998c3d20a3be2c',
        source=str(COLOR_SOURCE.relative_to(ROOT)),source_sha256=digest(COLOR_SOURCE),mesh_sources=provenance,
        topology='14 original revolute joints, four original connect constraints; no fixed merging of rear links',
        active_joints=ACTIVE,passive_joints=PASSIVE,marker_changes=marker_changes,assembled_joint_defaults=home,
        mass_inertia='All upstream parallel link mass and inertial tensors preserved; no serial URDF inertial substitution',
        appearance='Clean independent upstream link meshes; previously approved full body exterior reused as visual only',
        gas_spring='Original explicitly named qitanhuang sites; no inferred CAD-component deletion on clean parallel meshes',
        uncalibrated_spring=dict(stiffness_n_m=stiffness,damping_n_s_m=damping,free_length_m=free_length))
    root.set('model','fudan_parallel_no_gas_spring')
    paths=[]
    for enabled,filename in [(False,'robot_no_gas_spring.xml'),(True,'robot_gas_spring_damped.xml')]:
        variant=ET.fromstring(ET.tostring(root))
        if enabled:
            variant.set('model','fudan_parallel_compressible_gas_spring');tendon=ET.SubElement(variant,'tendon')
            for side in ['left','right']:
                spatial=ET.SubElement(tendon,'spatial',name=f'{side}_gas_spring',width='0.0075',rgba='0.65 0.65 0.65 1',
                    stiffness=str(stiffness),damping=str(damping),springlength=f'{free_length} 1',limited='false')
                for i in [0,1]:ET.SubElement(spatial,'site',site=f'{side}_lf{i}_qitanhuang')
        ET.indent(variant);path=OUTPUT/filename;ET.ElementTree(variant).write(path,encoding='utf-8',xml_declaration=True);paths.append(path)
    manifest['xml_sha256']={p.name:digest(p) for p in paths}
    (OUTPUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    for path in paths:
        model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        print(path,'hinges',model.njnt-1,'loops',model.neq,'max closure error',max(errors(model,data).values()))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--stiffness',type=float,default=1000.)
    parser.add_argument('--damping',type=float,default=10.);parser.add_argument('--free-length',type=float,default=.20)
    args=parser.parse_args();build(args.stiffness,args.damping,args.free_length)
