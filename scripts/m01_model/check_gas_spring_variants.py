"""Actually retract/extend both legs; validate configurable dissipative damping."""
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

os.environ['MUJOCO_GL']='egl'
os.environ['EGL_PLATFORM']='surfaceless'
os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
os.environ['__EGL_VENDOR_LIBRARY_FILENAMES']='/usr/share/glvnd/egl_vendor.d/50_mesa.json'

import mujoco
import numpy as np
from PIL import Image,ImageDraw

from prepare_model import ROOT,OUTPUT,ORDER,digest

FILES=['robot_no_gas_spring.xml','robot_gas_spring_damped.xml']
INITIAL=np.array([.2,.4,0,-.2,-.4,0.])


def load_fixture(filename):
    # Explicit diagnostic fixture: root fixed in air, gravity off.
    # Output XMLs themselves retain their free root and normal gravity.
    root=ET.parse(OUTPUT/filename).getroot()
    base=root.find('worldbody/body');base.remove(base.find('freejoint'));base.set('pos','0 0 0.6')
    root.find('option').set('gravity','0 0 0')
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'),
        assets={node.get('file'):(OUTPUT/node.get('file')).read_bytes() for node in root.findall('asset/mesh')})


def reset(model):
    data=mujoco.MjData(model)
    for name,value in zip(ORDER,INITIAL):data.qpos[model.joint(name).qposadr[0]]=value
    mujoco.mj_forward(model,data)
    return data


def metrics(model,data):
    mujoco.mj_forward(model,data)
    return dict(knees_rad=[float(data.qpos[model.joint(n).qposadr[0]]) for n in ['lf1_Joint','rf1_Joint']],
        base_to_wheel_z_m=[float(data.xpos[model.body('base_link').id,2]-data.xpos[model.body(n).id,2]) for n in ['l_wheel_Link','r_wheel_Link']],
        spring_length_m=data.ten_length.tolist())


def main():
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    output=ROOT/'runs/m01_model'/f'gas_spring_check_{stamp}';output.mkdir(parents=True)
    models=[load_fixture(f) for f in FILES]
    datas=[reset(m) for m in models]
    indexes=[[(int(m.joint(n).qposadr[0]),int(m.joint(n).dofadr[0])) for n in ORDER] for m in models]
    results=[dict(file=f,initial=metrics(m,d),max_torque_nm=0.,finite=True) for f,m,d in zip(FILES,models,datas)]
    camera=mujoco.MjvCamera();camera.lookat[:]=[.04,0,.68];camera.distance=1.5;camera.azimuth=135;camera.elevation=-15
    options=mujoco.MjvOption();options.geomgroup[2]=1;options.geomgroup[3]=0;options.sitegroup[:]=0
    renderers=[mujoco.Renderer(m,height=480,width=640) for m in models]
    encoder=subprocess.Popen(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
        '-s','1280x480','-r','25','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(output/'retract_extend_comparison.mp4')],stdin=subprocess.PIPE)
    traces=[[],[]]
    for step in range(2001):
        t=step*.002
        # Rest .2 s, retract smoothly through 1.2 s, hold, then extend and hold.
        if t<.2:blend=0.
        elif t<1.2:blend=(1-np.cos(np.pi*(t-.2)))/2
        elif t<2.:blend=1.
        elif t<3.:blend=(1+np.cos(np.pi*(t-2.)))/2
        else:blend=0.
        target=INITIAL.copy();target[1]+=.4*blend;target[4]-=.4*blend
        for i,(m,d,ix,result) in enumerate(zip(models,datas,indexes,results)):
            q=np.array([d.qpos[a] for a,b in ix]);v=np.array([d.qvel[b] for a,b in ix])
            torque=80*(target-q)-3*v
            torque[[2,5]]=-.2*v[[2,5]]
            torque=np.clip(torque,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
            d.ctrl[:]=torque
            result['max_torque_nm']=max(result['max_torque_nm'],float(abs(torque).max()))
            if step==999:result['retracted']=metrics(m,d)
            if step%20==0:traces[i].append([d.time,*d.qpos,*d.ten_length])
        if step%20==0:
            pictures=[]
            for i,(m,d,renderer) in enumerate(zip(models,datas,renderers)):
                renderer.update_scene(d,camera=camera,scene_option=options)
                picture=Image.fromarray(renderer.render());draw=ImageDraw.Draw(picture)
                draw.rectangle([0,0,640,65],fill='black')
                label='NO gas spring' if i==0 else 'Compressible gas spring | k=1000 N/m, c=10 N*s/m'
                draw.text((8,6),label,fill='white')
                draw.text((8,25),f't={t:.2f}s | root fixed / gravity off / diagnostic PD',fill='white')
                if i:draw.text((8,44),'Length: '+', '.join(f'{x*1000:.1f} mm' for x in d.ten_length),fill='white')
                pictures.append(np.asarray(picture))
            frame=np.concatenate(pictures,axis=1)
            if step in [0,1000,2000]:Image.fromarray(frame).save(output/{0:'initial.png',1000:'retracted.png',2000:'extended.png'}[step])
            encoder.stdin.write(frame.tobytes())
        if step<2000:
            for m,d in zip(models,datas):mujoco.mj_step(m,d)
    encoder.stdin.close()
    if encoder.wait()!=0:raise RuntimeError('ffmpeg failed')
    for renderer in renderers:renderer.close()
    for i,(m,d,result) in enumerate(zip(models,datas,results)):
        result['extended']=metrics(m,d)
        result['finite']=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
        result['warnings']=[int(w.number) for w in d.warning]
        result['leg_retraction_m']=(np.array(result['initial']['base_to_wheel_z_m'])-result['retracted']['base_to_wheel_z_m']).tolist()
        result['passed']=bool(result['finite'] and not any(result['warnings']) and min(result['leg_retraction_m'])>.04
            and np.max(abs(np.array(result['retracted']['knees_rad'])-[.8,-.8]))<.04
            and np.max(abs(np.array(result['extended']['knees_rad'])-[.4,-.4]))<.04)
        np.save(output/f'{Path(FILES[i]).stem}_trace.npy',np.array(traces[i]))
    # Demonstrate XML-equivalent damping changes, with spring force isolated out.
    m=models[1];d=reset(m);m.tendon_stiffness[:]=0
    d.qvel[m.joint('lf1_Joint').dofadr[0]]=.7
    d.qvel[m.joint('rf1_Joint').dofadr[0]]=-.7
    damping=[]
    for c in [0.,10.,50.]:
        m.tendon_damping[:]=c;mujoco.mj_forward(m,d)
        damping.append(dict(damping_n_s_m=c,passive_power_w=float(d.qfrc_passive@d.qvel),
            expected_power_w=float(-c*(d.ten_velocity@d.ten_velocity)),tendon_velocity_m_s=d.ten_velocity.tolist()))
    xml_info=[]
    for f in FILES:
        m=mujoco.MjModel.from_xml_path(str(OUTPUT/f))
        xml_info.append(dict(file=f,sha256=digest(OUTPUT/f),nq=m.nq,nv=m.nv,nu=m.nu,neq=m.neq,ntendon=m.ntendon,mass_kg=float(m.body_mass.sum())))
    report=dict(results=results,damping_test=damping,xml_info=xml_info,python=sys.executable,mujoco=mujoco.__version__,
        command=[sys.executable,'-B',str(Path(__file__).resolve())],fixture='Root fixed in air, zero gravity, diagnostic PD80/3; not balance or locomotion',
        script_sha256=digest(Path(__file__).resolve()),passed=all(x['passed'] for x in results)
        and all(abs(x['passive_power_w']-x['expected_power_w'])<1e-10 for x in damping)
        and damping[-1]['passive_power_w']<damping[1]['passive_power_w']<0.)
    (output/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(output);print(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
