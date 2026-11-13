from pathlib import Path
import sys,json,numpy as np,mujoco
from scipy.optimize import least_squares
root=Path(__file__).resolve().parents[2];m=mujoco.MjModel.from_xml_path(str(root/'assets/robot_no_gas_spring.xml'));d=mujoco.MjData(m);home=json.loads((root/'standing_pose.json').read_text())['no-gas'];qhome=np.array(home['qpos']);ids=[2]+[int(m.jnt_qposadr[j]) for j in range(1,m.njnt) if 'wheel' not in m.joint(j).name];bounds=(np.r_[.13,np.full(len(ids)-1,-6),m.actuator_ctrlrange[:,0]],np.r_[.5,np.full(len(ids)-1,6),m.actuator_ctrlrange[:,1]])
pairs=[(m.site(f'{s}_front_site{i}').id,m.site(f'{s}_rear_site{i}').id) for s in ('Left','Right') for i in (1,2)]
def residual(x,L):
 d.qpos[:]=qhome;d.qpos[ids]=x[:len(ids)];d.qvel[:]=0;d.ctrl[:]=x[len(ids):];mujoco.mj_forward(m,d)
 closure=np.concatenate([d.site_xpos[a]-d.site_xpos[b] for a,b in pairs]);lengths=[np.linalg.norm(d.xpos[m.body(s+'_wheel_Link').id]-d.xanchor[m.joint(s+'f0_Joint').id]) for s in ('l','r')]
 return np.r_[closure*1000,(np.array(lengths)-L)*1000,d.qacc*.02]
points=[]
for L in np.linspace(.18,.34,9):
 # IK uses branch near previously validated standing geometry.
 base=json.loads((Path(__file__).parent/'baseline_ramp/points.json').read_text());nearest=min([p for p in base if p['length']<.29],key=lambda p:abs(p['length']-L));start=np.r_[np.array(nearest['qpos'])[ids],nearest['ctrl']];start=np.clip(start,bounds[0]+1e-8,bounds[1]-1e-8)
 sol=least_squares(residual,start,args=(L,),bounds=bounds,xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=800,x_scale='jac');res=residual(sol.x,L);p=dict(length=float(L),qpos=d.qpos.tolist(),ctrl=d.ctrl.tolist(),max_qacc=float(abs(d.qacc).max()),max_residual=float(abs(res).max()),nfev=sol.nfev,success=bool(sol.success));points.append(p);print(L,p['max_qacc'],p['max_residual'],sol.nfev,flush=True)
(Path(__file__).parent/'static_points.json').write_text(json.dumps(points,indent=2))
