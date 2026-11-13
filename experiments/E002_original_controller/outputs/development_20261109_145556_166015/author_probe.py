import mujoco as mj,numpy as np
from scipy.linalg import solve_continuous_are
m=mj.MjModel.from_xml_path('/home/cofallen/Code/毕业设计/assets/robots/infantry_parallel/robot_no_gas_spring.xml');d=mj.MjData(m)
z=np.load('/home/cofallen/Code/毕业设计/runs/m02_standing/lqr_no-gas_20261109_135714_434732/design.npz');qref=z['qref'];uref=z['uref'];d.qpos[:]=qref;mj.mj_forward(m,d)
active=[m.joint(n).dofadr[0] for n in ['lf0_Joint','l20_Joint','rf0_Joint','r20_Joint']];passive=[i for i in range(6,20) if i not in active+[8,11]]
def measure():
 J=np.array(d.efc_J).reshape(-1,20)[d.efc_type==mj.mjtConstraint.mjCNSTR_EQUALITY];N=np.zeros((20,4));N[active]=np.eye(4);N[passive]=-np.linalg.pinv(J[:,passive])@J[:,active]
 vals=[];Ls=[];Ds=[];Cs=[];contacts=[]
 for side,k in [('l',0),('r',2)]:
  wheel=m.body(side+'_wheel_Link').id;hip=d.xanchor[m.joint(side+'f0_Joint').id];jp=np.zeros((3,20));jr=jp.copy();hj=jp.copy();hr=jp.copy();mj.mj_jacBody(m,d,jp,jr,wheel);mj.mj_jac(m,d,hj,hr,hip,1)
  r=d.xpos[wheel]-hip;rel=jp-hj;L=np.linalg.norm(r);theta=np.arctan2(-r[0],-r[2]);dl=r@rel/L;ct=(r[2]*rel[0]-r[0]*rel[2])/(r[0]**2+r[2]**2)
  vals.append(theta);Ls.append(L);Ds.append(np.vstack([dl,ct])@N[:,k:k+2]);Cs.append(ct)
  cross=np.array([[0,.06,0],[-.06,0,0],[0,0,0]]) # skew rcontact=(0,0,-.06)
  contacts.append(jp-cross@jr)
 C=np.vstack([np.mean(Cs,axis=0),np.eye(20)[0],-np.eye(20)[4]])
 return np.array(vals),np.array(Ls),Ds,C,J,contacts
th,Lref,D,C,J,contacts=measure();basis=np.eye(20);constraint=np.vstack([J,*contacts,*[DUMMY for DUMMY in []],basis[3],basis[5]])
# length derivatives from each leg across all q, active dependency equivalent constraints row N inverse? get direct length separately
lengthJ=[]
for side in ['l','r']:
 b=m.body(side+'_wheel_Link').id;hip=d.xanchor[m.joint(side+'f0_Joint').id];jp=np.zeros((3,20));jr=jp.copy();hj=jp.copy();mj.mj_jacBody(m,d,jp,jr,b);mj.mj_jac(m,d,hj,jr,hip,1);r=d.xpos[b]-hip;lengthJ.append(r@(jp-hj)/np.linalg.norm(r))
constraint=np.vstack([constraint,*lengthJ]); M=np.vstack([constraint,C]);target=np.vstack([np.zeros((len(constraint),3)),np.eye(3)]);T=np.linalg.lstsq(M,target,rcond=None)[0];print('rank',np.linalg.matrix_rank(constraint),'err',np.max(abs(M@T-target)), 'D',D,'theta',th, 'L',Lref)
bias=[np.linalg.solve(D[i].T,uref[[0,4] if i==0 else [2,5]]) for i in range(2)];print('bias',bias)
def torques(Tw,Tp):
 th,L,D,C,J,cont=measure();out=np.zeros(6)
 for i in range(2):
  F=bias[i][0]+2000*(Lref[i]-L[i])-80*(lengthJ[i]@d.qvel)
  out[[0,4] if i==0 else [2,5]]=D[i].T@np.array([F,bias[i][1]+Tp])
 out[1]=Tw;out[3]=-Tw;return out
def f(pos,vel,u):
 d.qpos[:]=qref;mj.mj_integratePos(m,d.qpos,T@pos,1);d.qvel[:]=T@vel;mj.mj_forward(m,d);d.ctrl[:]=torques(*u);mj.mj_forward(m,d);return C@d.qacc
A=np.zeros((6,6));B=np.zeros((6,2));A[[0,2,4],[1,3,5]]=1;eps=1e-5
for j in range(3):
 v=np.eye(3)[j]*eps;A[[1,3,5],j*2]=(f(v,np.zeros(3),[0,0])-f(-v,np.zeros(3),[0,0]))/(2*eps);A[[1,3,5],j*2+1]=(f(np.zeros(3),v,[0,0])-f(np.zeros(3),-v,[0,0]))/(2*eps)
for j in range(2):
 u=np.eye(2)[j]*eps;B[[1,3,5],j]=(f(np.zeros(3),np.zeros(3),u)-f(np.zeros(3),np.zeros(3),-u))/(2*eps)
P=solve_continuous_are(A,B,np.diag([3000,1,500,1,30000,1]),np.eye(2));K=B.T@P
print('A',A,'B',B,'K',K,'eig',np.linalg.eigvals(A-B@K))
d.qpos[:]=qref;d.qvel[:]=0;v=np.zeros(20);v[4]=.03;mj.mj_integratePos(m,d.qpos,v,1)
for step in range(10000):
 mj.mj_forward(m,d);th,L,D,C,J,cont=measure();pitch=2*np.arctan2(d.qpos[5],d.qpos[3]);state=np.array([th.mean()-np.mean(zero_theta),C[0]@d.qvel,d.qpos[0],d.qvel[0],-pitch,-d.qvel[4]]) if False else np.array([th.mean()-(-.010478),C[0]@d.qvel,d.qpos[0],d.qvel[0],-pitch,-d.qvel[4]])
 Tw,Tp=-K@state;d.ctrl[:]=np.clip(torques(Tw,Tp),m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);mj.mj_step(m,d)
 if step%1000==0:print(step/1000,d.qpos[:7],d.ctrl,flush=True)
 if abs(d.qpos[5])>.3:break
np.savez('/tmp/author_gain.npz',A=A,B=B,K=K,qref=qref,uref=uref,T=T)
