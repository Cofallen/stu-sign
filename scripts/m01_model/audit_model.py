"""Executable model audit, including negative findings; no GPU or policy needed."""
import csv
import datetime
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from prepare_model import ROOT, UPSTREAM, SOURCE, OUTPUT, ORDER, digest

NATIVE = UPSTREAM / 'mujoco/assert_now/infantry_binglian_yuntai/infantry_V2/meshes/mjmodel.xml'
NOMINAL = dict(zip(ORDER, [.2, .4, 0, -.2, -.4, 0]))


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))


def source_inventory():
    result = []
    for path in sorted(UPSTREAM.rglob('*.urdf')):
        root = ET.parse(path).getroot()
        links = []
        meshes = []
        for link in root.findall('link'):
            inertial = link.find('inertial')
            inertia = inertial.find('inertia')
            try:
                for value in inertia.attrib.values():
                    float(value)
            except ValueError as error:
                links.append(dict(name=link.get('name'), mass=float(inertial.find('mass').get('value')),
                                  inertia=inertia.attrib, parse_error=str(error)))
                continue
            t = np.array([[float(inertia.get('ixx')), float(inertia.get('ixy')), float(inertia.get('ixz'))],
                          [float(inertia.get('ixy')), float(inertia.get('iyy')), float(inertia.get('iyz'))],
                          [float(inertia.get('ixz')), float(inertia.get('iyz')), float(inertia.get('izz'))]])
            eig = np.linalg.eigvalsh(t)
            links.append(dict(name=link.get('name'), mass=float(inertial.find('mass').get('value')),
                              com=inertial.find('origin').attrib, inertia=inertia.attrib,
                              principal_inertia=eig.tolist(), positive_definite=bool(eig[0] > 0),
                              triangle_margin=float(eig[0] + eig[1] - eig[2])))
        for mesh in root.findall('.//mesh'):
            filename = mesh.get('filename')
            # Known project URDFs use relative or package paths. Resolve only declared package name.
            if filename.startswith('package://'):
                tail = filename[len('package://'):].split('/', 1)
                candidate = path.parent.parent / tail[1]
            else:
                candidate = path.parent / filename
            meshes.append(dict(reference=filename, exists=candidate.exists()))
        joints = [dict(j.attrib, parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                       origin=j.find('origin').attrib, axis=j.find('axis').attrib,
                       limit=j.find('limit').attrib) for j in root.findall('joint')]
        result.append(dict(path=str(path.relative_to(ROOT)), sha256=digest(path), total_mass=sum(l['mass'] for l in links),
                           links=links, joints=joints, meshes=meshes,
                           mimic_count=len(root.findall('.//mimic')),
                           transmission_count=len(root.findall('transmission'))))
    return result


def snapshot(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return dict(nq=model.nq, nv=model.nv, nu=model.nu, neq=model.neq,
                mass=float(model.body_mass.sum()), gravity=model.opt.gravity.tolist(), dt=model.opt.timestep,
                bodies=[dict(name=model.body(i).name, mass=float(model.body_mass[i]), com=model.body_ipos[i].tolist(),
                             principal_inertia=model.body_inertia[i].tolist(), inertia_quat_wxyz=model.body_iquat[i].tolist())
                        for i in range(1, model.nbody)],
                joints=[dict(name=model.joint(i).name, qpos_address=int(model.jnt_qposadr[i]), dof_address=int(model.jnt_dofadr[i]),
                             axis_local=model.jnt_axis[i].tolist(), axis_world_zero=data.xaxis[i].tolist(),
                             limited=bool(model.jnt_limited[i]), range=model.jnt_range[i].tolist(),
                             actuator_force_limited=bool(model.jnt_actfrclimited[i]),
                             actuator_force_range=model.jnt_actfrcrange[i].tolist(),
                             damping=float(model.dof_damping[model.jnt_dofadr[i]]),
                             armature=float(model.dof_armature[model.jnt_dofadr[i]])) for i in range(model.njnt)],
                actuators=[dict(name=model.actuator(i).name, transmission_type=int(model.actuator_trntype[i]),
                                target_id=int(model.actuator_trnid[i, 0]), gear=model.actuator_gear[i].tolist(),
                                ctrl_limited=bool(model.actuator_ctrllimited[i]), ctrl_range=model.actuator_ctrlrange[i].tolist())
                           for i in range(model.nu)],
                geoms=[dict(name=model.geom(i).name, body=model.body(model.geom_bodyid[i]).name, type=int(model.geom_type[i]),
                            contype=int(model.geom_contype[i]), conaffinity=int(model.geom_conaffinity[i]),
                            condim=int(model.geom_condim[i]), friction=model.geom_friction[i].tolist()) for i in range(model.ngeom)])


def loop_errors(model, data):
    return {model.equality(i).name: float(np.linalg.norm(data.site_xpos[model.eq_obj1id[i]] - data.site_xpos[model.eq_obj2id[i]]))
            for i in range(model.neq)}


def baseline(model, nominal=True):
    data = mujoco.MjData(model)
    data.qpos[:3] = [0, 0, 1.5]
    if nominal:
        for name, value in NOMINAL.items():
            data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)
    return data


def selected_tests(model, inventory, output):
    checks = {}
    # Check exact URDF tensors including off-diagonal terms against compiled MJCF.
    mass_error = 0.
    inertia_error = 0.
    for link in inventory['links']:
        bid = model.body(link['name']).id
        mass_error = max(mass_error, abs(model.body_mass[bid] - link['mass']))
        a = link['inertia']
        expected = np.array([[float(a['ixx']), float(a['ixy']), float(a['ixz'])],
                             [float(a['ixy']), float(a['iyy']), float(a['iyz'])],
                             [float(a['ixz']), float(a['iyz']), float(a['izz'])]])
        # Selected URDF has zero inertial rpy; reject if that changes.
        assert np.allclose(np.fromstring(link['com']['rpy'], sep=' '), 0)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, model.body_iquat[bid])
        rotation = rotation.reshape(3, 3)
        actual = rotation @ np.diag(model.body_inertia[bid]) @ rotation.T
        inertia_error = max(inertia_error, float(np.max(abs(actual - expected))))
    checks['inertial_preservation'] = dict(max_mass_error_kg=mass_error, max_tensor_error_kg_m2=inertia_error,
                                         pass_check=bool(mass_error < 1e-10 and inertia_error < 1e-7))
    data = baseline(model)
    start = data.subtree_com[1].copy()
    for _ in range(50):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    elapsed = data.time
    displacement = data.subtree_com[1] - start
    expected = model.opt.gravity * (elapsed**2 / 2 + elapsed * model.opt.timestep / 2)
    checks['freefall'] = dict(duration_s=elapsed, com_displacement_m=displacement.tolist(),
                              discrete_expected_m=expected.tolist(), contacts=data.ncon,
                              pass_check=bool(np.max(abs(displacement - expected)) < 2e-5 and data.ncon == 0))
    # Isolated signed torque pulses: no gravity and no ground contact, paired +/-.
    original_gravity = model.opt.gravity.copy()
    model.opt.gravity[:] = 0
    pulses = []
    for name in ORDER:
        jid = model.joint(name).id
        qadr = int(model.jnt_qposadr[jid]); vadr = int(model.jnt_dofadr[jid])
        aid = model.actuator(f'motor_{name}').id
        record = dict(joint=name, axis_world_nominal=None, impulse_torque_nm=.1, duration_s=.02)
        for sign, label in [(1, 'positive'), (-1, 'negative')]:
            data = baseline(model)
            record['axis_world_nominal'] = data.xaxis[jid].tolist()
            q0 = data.qpos[qadr]
            data.ctrl[aid] = sign * .1
            for _ in range(10):
                mujoco.mj_step(model, data)
            record[label] = dict(delta_rad=float(data.qpos[qadr] - q0), velocity_rad_s=float(data.qvel[vadr]))
        record['pass_check'] = record['positive']['delta_rad'] > 0 and record['negative']['delta_rad'] < 0
        pulses.append(record)
    checks['signed_pulses'] = pulses
    # Explicit torque saturation, including wheels' large source-declared limit.
    data = baseline(model)
    data.ctrl[:] = 1e6
    mujoco.mj_forward(model, data)
    checks['torque_clipping'] = dict(applied=data.actuator_force.tolist(), limits=model.actuator_ctrlrange[:, 1].tolist(),
                                    pass_check=bool(np.allclose(data.actuator_force, model.actuator_ctrlrange[:, 1])))
    # Probe outside each bounded joint limit; solver must oppose the violation.
    limits = []
    for name in ORDER:
        joint = model.joint(name)
        if not model.jnt_limited[joint.id]:
            continue
        for edge, sign in [(0, -1), (1, 1)]:
            data = baseline(model)
            data.qpos[joint.qposadr[0]] = model.jnt_range[joint.id, edge] + sign * .02
            mujoco.mj_forward(model, data)
            force = float(data.qfrc_constraint[joint.dofadr[0]])
            limits.append(dict(joint=name, edge=edge, restoring_force_nm=force, pass_check=bool(force * sign < 0)))
    checks['joint_limits'] = limits
    model.opt.gravity[:] = original_gravity
    # Place at first contact by querying actual collision distances, not guessing wheel radius.
    data = baseline(model)
    # At z=0 wheel-floor contacts are guaranteed for this particular model/pose.
    data.qpos[2] = 0
    mujoco.mj_forward(model, data)
    wheel_contacts = [c for c in data.contact if any('wheel' in model.body(model.geom_bodyid[g]).name for g in c.geom)]
    if not wheel_contacts:
        raise RuntimeError('No wheel contact available to establish nominal support height')
    height = -min(c.dist for c in wheel_contacts)
    data.qpos[2] = height + .001
    mujoco.mj_forward(model, data)
    checks['nominal_geometry'] = dict(base_height_at_wheel_contact_m=height,
                                      wheel_center_positions_m={n: data.xpos[model.body(n).id].tolist() for n in ['l_wheel_Link', 'r_wheel_Link']},
                                      initial_contacts=data.ncon)
    # Passive settling is a contact test, not a balance controller.
    trajectory = []
    min_distance = 0.
    wheel_contact_steps = 0
    self_contact_steps = 0
    support_forces = []
    max_speed_tail = 0.
    max_base_linear_speed_tail = 0.
    max_base_angular_speed_tail = 0.
    min_distance_tail = 0.
    for step in range(2500):
        mujoco.mj_step(model, data)
        if step % 10 == 0:
            mujoco.mj_forward(model, data)
            trajectory.append([data.time, *data.qpos[:7], float(np.linalg.norm(data.qvel)), data.ncon])
        force_z = 0.
        wheel = False
        for c in data.contact:
            min_distance = min(min_distance, float(c.dist))
            bodies = [model.body(model.geom_bodyid[g]).name for g in c.geom]
            wheel |= any('wheel' in b for b in bodies)
            self_contact_steps += int(all(model.geom_bodyid[g] != 0 for g in c.geom))
        wheel_contact_steps += wheel
        if step >= 2000:
            max_speed_tail = max(max_speed_tail, float(np.linalg.norm(data.qvel)))
            max_base_linear_speed_tail = max(max_base_linear_speed_tail, float(np.linalg.norm(data.qvel[:3])))
            max_base_angular_speed_tail = max(max_base_angular_speed_tail, float(np.linalg.norm(data.qvel[3:6])))
            min_distance_tail = min(min_distance_tail, min((float(c.dist) for c in data.contact), default=0.))
            # Global total support force from contacts on world (equal and opposite).
            for i, c in enumerate(data.contact):
                if model.geom_bodyid[c.geom[0]] == 0 or model.geom_bodyid[c.geom[1]] == 0:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, i, force)
                    global_force = c.frame.reshape(3, 3).T @ force[:3]
                    force_z += abs(global_force[2])
            support_forces.append(force_z)
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    checks['passive_contact'] = dict(duration_s=data.time, finite=finite, min_contact_distance_m=min_distance,
                                     wheel_contact_step_fraction=wheel_contact_steps / 2500,
                                     self_contact_count=self_contact_steps, final_base_xyz=data.qpos[:3].tolist(),
                                     final_base_quat_wxyz=data.qpos[3:7].tolist(),
                                     max_speed_last_second=max_speed_tail,
                                     max_base_linear_speed_last_second_m_s=max_base_linear_speed_tail,
                                     max_base_angular_speed_last_second_rad_s=max_base_angular_speed_tail,
                                     min_contact_distance_last_second_m=min_distance_tail,
                                     mean_support_force_last_second_n=float(np.mean(support_forces)),
                                     robot_weight_n=float(model.body_mass.sum()*9.81),
                                     warning_counts=[int(w.number) for w in data.warning],
                                     note='Passive final contact may be on body; this is not upright balancing')
    mujoco.mj_forward(model, data)
    checks['passive_contact']['final_contact_bodies'] = sorted({model.body(model.geom_bodyid[g]).name for c in data.contact for g in c.geom})
    # Static mesh collision is soft. Keep both the transient and settled depth visible.
    checks['passive_contact']['support_force_gate'] = bool(finite and min_distance_tail > -.001
        and abs(np.mean(support_forces) / (model.body_mass.sum()*9.81) - 1) < .02)
    checks['passive_contact']['static_rest_gate'] = bool(max_base_linear_speed_tail < .01 and max_base_angular_speed_tail < .05)
    checks['passive_contact']['transient_3mm_gate'] = bool(min_distance > -.003)
    # Height feasibility is kinematic only; no claim of controlled height tracking.
    checks['symmetric_pose_scan'] = []
    for hip, knee in [(0., 0.), (.2, .4), (-.2, -.4)]:
        probe = baseline(model)
        for name, value in dict(lf0_Joint=hip, lf1_Joint=knee, rf0_Joint=-hip, rf1_Joint=-knee).items():
            probe.qpos[model.joint(name).qposadr[0]] = value
        probe.qpos[2] = 0
        mujoco.mj_forward(model, probe)
        contacts = [c for c in probe.contact if any('wheel' in model.body(model.geom_bodyid[g]).name for g in c.geom)]
        height = -min(c.dist for c in contacts)
        probe.qpos[2] = height + .001
        mujoco.mj_forward(model, probe)
        checks['symmetric_pose_scan'].append(dict(left_hip_rad=hip,left_knee_rad=knee,
            base_height_wheel_contact_m=height, other_contacts_after_clearance=probe.ncon))
    np.savetxt(output / 'passive_trajectory.csv', trajectory, delimiter=',',
               header='time,x,y,z,qw,qx,qy,qz,speed_norm,ncontact', comments='')
    return checks


def fixture_static_test():
    """Lock the six hinges and base XY/orientation, leaving vertical motion free.

    This explicit test fixture verifies normal contact equilibrium, never balance.
    """
    root = ET.parse(OUTPUT / 'robot.xml').getroot()
    base = root.find('worldbody/body')
    base.set('pos', '0 0 0')
    base.remove(base.find('freejoint'))
    ET.SubElement(base, 'joint', name='fixture_vertical', type='slide', axis='0 0 1', limited='false')
    for body in root.findall('.//body'):
        for joint in list(body.findall('joint')):
            name = joint.get('name')
            if name == 'fixture_vertical':
                continue
            q = NOMINAL[name]
            offset_rotation = np.array([np.cos(q/2), 0, 0, np.sin(q/2)])
            result = np.zeros(4)
            mujoco.mju_mulQuat(result, np.fromstring(body.get('quat', '1 0 0 0'), sep=' '), offset_rotation)
            body.set('quat', ' '.join(map(str, result)))
            body.remove(joint)
    root.remove(root.find('actuator'))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'),
        assets={p.name:p.read_bytes() for p in OUTPUT.glob('*.stl')})
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    wheels = [c for c in data.contact if any('wheel' in model.body(model.geom_bodyid[g]).name for g in c.geom)]
    data.qpos[0] = -min(c.dist for c in wheels) + .001
    forces = []; min_distance = 0.; max_speed = 0.
    for _ in range(2500):
        mujoco.mj_step(model, data)
        if data.time > 4:
            max_speed = max(max_speed, abs(float(data.qvel[0])))
            min_distance = min(min_distance, min((float(c.dist) for c in data.contact), default=0.))
            support = 0.
            for i, c in enumerate(data.contact):
                f = np.zeros(6)
                mujoco.mj_contactForce(model, data, i, f)
                support += abs((c.frame.reshape(3, 3).T @ f[:3])[2])
            forces.append(support)
    weight = float(model.body_mass.sum()*9.81)
    return dict(fixture='base XY and orientation fixed; all six joint angles fixed; base Z free',
                total_mass_kg=float(model.body_mass.sum()), mean_support_n=float(np.mean(forces)), weight_n=weight,
                max_vertical_speed_last_second_m_s=max_speed, min_contact_distance_last_second_m=min_distance,
                pass_check=bool(max_speed < 1e-4 and min_distance > -.001 and abs(np.mean(forces)/weight-1)<.01))


def main():
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'runs/m01_model' / f'model_audit_{stamp}'
    output.mkdir(parents=True)
    inventory = source_inventory()
    save(output / 'urdf_inventory.json', inventory)
    original_loads = {}
    for label, path in [('native_parallel', NATIVE), ('standard_urdf', SOURCE),
                         ('increased_urdf', SOURCE.with_name('infantry_V4_increase.urdf'))]:
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            original_loads[label] = dict(loaded=True, model=snapshot(model))
            if model.neq:
                data = mujoco.MjData(model); mujoco.mj_forward(model, data)
                original_loads[label]['initial_loop_distance_m'] = loop_errors(model, data)
                for _ in range(250):
                    mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                original_loads[label]['after_half_second_loop_distance_m'] = loop_errors(model, data)
                original_loads[label]['warning_counts'] = [int(w.number) for w in data.warning]
        except ValueError as error:
            original_loads[label] = dict(loaded=False, error=str(error))
    save(output / 'upstream_loads.json', original_loads)
    model = mujoco.MjModel.from_xml_path(str(OUTPUT / 'robot.xml'))
    save(output / 'compiled_model.json', snapshot(model))
    selected = next(x for x in inventory if x['path'] == str(SOURCE.relative_to(ROOT)))
    checks = selected_tests(model, selected, output)
    checks['static_contact_fixture'] = fixture_static_test()
    save(output / 'physics_checks.json', checks)
    with (output / 'joint_mapping.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['action_index','joint','joint_id','qpos_address','dof_address','motor_id','axis_world_nominal',
                         'angle_limited','lower_rad','upper_rad','source_effort_nm','source_velocity_rad_s','continuous_in_mjcf'])
        for i, name in enumerate(ORDER):
            joint = model.joint(name)
            source = next(j for j in selected['joints'] if j['name'] == name)
            axis = checks['signed_pulses'][i]['axis_world_nominal']
            writer.writerow([i,name,joint.id,int(joint.qposadr[0]),int(joint.dofadr[0]),i,axis,
                             bool(model.jnt_limited[joint.id]),*model.jnt_range[joint.id],
                             source['limit']['effort'],source['limit']['velocity'],not bool(model.jnt_limited[joint.id])])
    metadata = dict(python=sys.version, mujoco=mujoco.__version__, executable=sys.executable,
                    command=[sys.executable,'-B',str(Path(__file__).resolve())],
                    upstream_commit=subprocess.check_output(['git','-C',str(UPSTREAM),'rev-parse','HEAD'],text=True).strip(),
                    source_sha256=digest(SOURCE), generated_xml_sha256=digest(OUTPUT/'robot.xml'),
                    dependencies={name:importlib.metadata.version(name) for name in ['mujoco','numpy','scipy','trimesh','pillow']},
                    scripts_sha256={p.name:digest(p) for p in [Path(__file__).resolve(),Path(__file__).with_name('prepare_model.py')]})
    save(output / 'run_metadata.json', metadata)
    gates = dict(inertia=checks['inertial_preservation']['pass_check'], freefall=checks['freefall']['pass_check'],
        signed_pulses=all(x['pass_check'] for x in checks['signed_pulses']),
        limits=all(x['pass_check'] for x in checks['joint_limits']), torque_clip=checks['torque_clipping']['pass_check'],
        static_fixture=checks['static_contact_fixture']['pass_check'],
        contact_depth=checks['passive_contact']['transient_3mm_gate'],
        no_numerical_warnings=not any(checks['passive_contact']['warning_counts']))
    save(output/'acceptance.json', dict(scope='standard V4 simplified CPU smoke only', gates=gates,
        passed=all(gates.values()), excluded=['parallel mechanism acceptance','upright balance','mjlab GPU','Isaac Lab']))
    print(output)
    print(json.dumps(checks, ensure_ascii=False, indent=2))
    if not all(gates.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
