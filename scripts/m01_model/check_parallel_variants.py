"""Verify closed-loop mobility by actuating only the six physical motor axes."""
import argparse
import csv
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

os.environ['MUJOCO_GL']='egl';os.environ['EGL_PLATFORM']='surfaceless'
os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
os.environ['__EGL_VENDOR_LIBRARY_FILENAMES']='/usr/share/glvnd/egl_vendor.d/50_mesa.json'
import mujoco
import numpy as np
from PIL import Image,ImageDraw

from build_parallel_variants import OUTPUT,ACTIVE,PASSIVE,errors
from prepare_model import ROOT,COLOR_SOURCE,digest

FILES=['robot_no_gas_spring.xml','robot_gas_spring_damped.xml']


def fixture(filename):
    root=ET.parse(OUTPUT/filename).getroot();base=root.find('worldbody/body')
    base.remove(base.find('freejoint'));base.set('pos','0 0 0.6');root.find('option').set('gravity','0 0 0')
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'),
        assets={a.get('file'):(OUTPUT/a.get('file')).read_bytes() for a in root.findall('asset/mesh')})


def target(t):
    # Front/rear leg motors move differentially; rear links follow constraints.
    if t<.3:a=0.
    elif t<1.3:a=-.25*(1-np.cos(np.pi*(t-.3)))/2
    elif t<2.:a=-.25
    elif t<3.:a=-.25*(1+np.cos(np.pi*(t-2.)))/2
    else:a=0.
    return dict(lf0_Joint=a,l20_Joint=-a,rf0_Joint=-a,r20_Joint=a,
        l_wheel_Joint=.3*np.sin(2*np.pi*t/4),r_wheel_Joint=-.3*np.sin(2*np.pi*t/4))


def main(video=False):
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out=ROOT/'runs/m01_model'/f'parallel_check_{stamp}';out.mkdir(parents=True)
    models=[fixture(f) for f in FILES];datas=[mujoco.MjData(m) for m in models]
    records=[];traces=[[],[]]
    for m,d,f in zip(models,datas,FILES):
        mujoco.mj_forward(m,d)
        records.append(dict(file=f,startup_closure_m=errors(m,d),max_closure_error_m=0.,
            max_passive_actuator_torque_nm=0.,max_motor_torque_nm=0.,initial_wheel_z=d.xpos[m.body('l_wheel_Link').id,2],
            min_tendon_length_m=d.ten_length.tolist(),max_tendon_length_m=d.ten_length.tolist()))
    encoder=None;renderers=[]
    if video:
        cam=mujoco.MjvCamera();cam.lookat[:]=[.03,0,.68];cam.distance=1.5;cam.azimuth=135;cam.elevation=-15
        options=mujoco.MjvOption();options.geomgroup[3]=0;options.sitegroup[:]=0
        renderers=[mujoco.Renderer(m,height=480,width=640) for m in models]
        encoder=subprocess.Popen(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
            '-s','1280x480','-r','20','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(out/'parallel_retraction.mp4')],stdin=subprocess.PIPE)
    for step in range(4001):
        time=step*.001;goal=target(time)
        for i,(m,d,r) in enumerate(zip(models,datas,records)):
            mujoco.mj_forward(m,d)
            r['max_closure_error_m']=max(r['max_closure_error_m'],max(errors(m,d).values()))
            for aid in range(m.nu):
                jid=int(m.actuator_trnid[aid,0]);name=m.joint(jid).name
                q=float(d.qpos[m.jnt_qposadr[jid]]);v=float(d.qvel[m.jnt_dofadr[jid]])
                kp,kd=(.5,.05) if 'wheel' in name else (80.,3.)
                d.ctrl[aid]=np.clip(kp*(goal[name]-q)-kd*v,*m.actuator_ctrlrange[aid])
            passive=[int(m.joint(n).dofadr[0]) for n in PASSIVE]
            r['max_passive_actuator_torque_nm']=max(r['max_passive_actuator_torque_nm'],float(abs(d.qfrc_actuator[passive]).max()))
            r['max_motor_torque_nm']=max(r['max_motor_torque_nm'],float(abs(d.ctrl).max()))
            if m.ntendon:
                r['min_tendon_length_m']=np.minimum(r['min_tendon_length_m'],d.ten_length).tolist()
                r['max_tendon_length_m']=np.maximum(r['max_tendon_length_m'],d.ten_length).tolist()
            if step==2000:
                r['wheel_z_retraction_m']=float(d.xpos[m.body('l_wheel_Link').id,2]-r['initial_wheel_z'])
                r['retracted_motor_angles_rad']={n:float(d.qpos[m.joint(n).qposadr[0]]) for n in ACTIVE if 'wheel' not in n}
            if step%10==0:traces[i].append([d.time,*d.qpos,*d.ten_length])
        if video and step%50==0:
            pictures=[]
            for i,(m,d,renderer) in enumerate(zip(models,datas,renderers)):
                renderer.update_scene(d,camera=cam,scene_option=options);p=Image.fromarray(renderer.render());draw=ImageDraw.Draw(p)
                draw.rectangle([0,0,640,62],fill='black')
                draw.text((8,5),'PARALLEL: 14 hinges / 4 closed connections',fill='white')
                draw.text((8,24),('No gas spring' if i==0 else f'Gas spring: c={m.tendon_damping[0]:g} N*s/m')+f' | t={time:.2f}s',fill='white')
                draw.text((8,43),'Root fixed / gravity off; passive joints are NOT commanded',fill='white');pictures.append(np.asarray(p))
            frame=np.concatenate(pictures,axis=1);encoder.stdin.write(frame.tobytes())
            if step in [0,2000,4000]:Image.fromarray(frame).save(out/{0:'initial.png',2000:'retracted.png',4000:'returned.png'}[step])
        if step<4000:
            for m,d in zip(models,datas):mujoco.mj_step(m,d)
    if encoder:
        encoder.stdin.close()
        if encoder.wait()!=0:raise RuntimeError('ffmpeg failed')
        for r in renderers:r.close()
    for i,(m,d,r) in enumerate(zip(models,datas,records)):
        values=np.array(traces[i]);np.save(out/f'{Path(FILES[i]).stem}_trace.npy',values)
        ranges=np.ptp(values[:,1:1+m.nq],axis=0)
        r['joint_rotation_range_rad']={m.joint(j).name:float(ranges[m.jnt_qposadr[j]]) for j in range(m.njnt)}
        r['warnings']=[int(w.number) for w in d.warning]
        r['finite']=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
        r['passed']=r['finite'] and not any(r['warnings']) and r['max_closure_error_m']<.0002 and min(r['joint_rotation_range_rad'].values())>.001 and r['max_passive_actuator_torque_nm']==0 and r['wheel_z_retraction_m']>.025
        r['joint_count']=m.njnt;r['connect_count']=m.neq
        # Original delivered free-root XML: normal gravity smoke, no fixture.
        free=mujoco.MjModel.from_xml_path(str(OUTPUT/FILES[i]));fd=mujoco.MjData(free)
        for _ in range(1000):mujoco.mj_step(free,fd)
        r['free_root_smoke']=dict(finite=bool(np.isfinite(fd.qpos).all()),warnings=[int(w.number) for w in fd.warning],nq=free.nq,nv=free.nv,nu=free.nu,mass_kg=float(free.body_mass.sum()))
    with (out/'joint_mapping.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['joint','role','local_axis','rotation_range_no_spring_rad','rotation_range_spring_rad'])
        for j in range(models[0].njnt):
            name=models[0].joint(j).name;writer.writerow([name,'active' if name in ACTIVE else 'passive',models[0].jnt_axis[j].tolist(),*[r['joint_rotation_range_rad'][name] for r in records]])
    report=dict(results=records,passed=all(r['passed'] for r in records),mujoco=mujoco.__version__,python=sys.executable,
        xml_sha256={f:digest(OUTPUT/f) for f in FILES},script_sha256=digest(Path(__file__).resolve()),
        fixture='Root fixed, zero gravity, only six physical motor axes commanded; all eight passive hinges unactuated',
        notes='Each hinge rotates within linkage coupling; hinges are not independently free due to four closed pin joints')
    (out/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(out);print(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--video',action='store_true');args=p.parse_args();main(args.video)
