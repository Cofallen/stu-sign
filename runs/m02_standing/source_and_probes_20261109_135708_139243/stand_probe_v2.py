import mujoco as mj, numpy as np
from scipy.optimize import least_squares
m=mj.MjModel.from_xml_path('/home/cofallen/Code/毕业设计/assets/robots/infantry_parallel/robot_no_gas_spring.xml');d=mj.MjData(m)
wheel=[m.body(n).id for n in ['l_wheel_Link','r_wheel_Link']]
pairs=[(m.site(a).id,m.site(b).id) for a,b in [('Left_front_site1','Left_rear_site1'),('Left_front_site2','Left_rear_site2'),('Right_front_site1','Right_rear_site1'),('Right_front_site2','Right_rear_site2')]]
jids=[j for j in range(1,m.njnt) if 'wheel' not in m.joint(j).name];qa=[m.jnt_qposadr[j] for j in jids]
q0=m.qpos0.copy()
def geom(x):
 d.qpos[:]=q0;d.qpos[2]=x[0];d.qpos[qa]=x[1:];mj.mj_forward(m,d)
 return np.r_[np.concatenate([d.site_xpos[a]-d.site_xpos[b] for a,b in pairs])*20, (d.xpos[wheel,0]-d.subtree_com[1,0])*5,(d.xpos[wheel,2]-.0598)*5,(x[0]-.25)*5, (x[1:]-q0[qa])*.001]
x=least_squares(geom,np.r_[.25,q0[qa]],xtol=1e-13,ftol=1e-13,gtol=1e-13,max_nfev=300)
print('geom',x.cost,x.optimality);geom(x.x);print('q',d.qpos);print('wheel',d.xpos[wheel], 'com',d.subtree_com[1]);target=d.qpos.copy()
# static acceleration equilibrium: vary z, all non-wheel coordinates, inputs, pitch
n=len(qa)
def static(x):
 d.qpos[:]=target;d.qpos[2]=x[0];d.qpos[qa]=x[1:n+1];d.qpos[3:7]=[np.cos(x[n+1]/2),0,np.sin(x[n+1]/2),0];d.qvel[:]=0;d.ctrl[:]=x[n+2:];mj.mj_forward(m,d)
 return np.r_[d.qacc*.01,(d.qpos[qa]-target[qa])*.01,x[n+1]*.1]
s=least_squares(static,np.r_[target[2],target[qa],0.,np.zeros(6)],jac='3-point',x_scale='jac',xtol=1e-12,ftol=1e-12,gtol=1e-11,max_nfev=1000)
static(s.x);print('static',s.cost,s.optimality,s.message);print('q',d.qpos,'u',d.ctrl,'acc',d.qacc,'contacts',d.ncon)
np.savez('/tmp/stand_eq.npz',q=d.qpos,u=d.ctrl)
