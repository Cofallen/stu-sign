exec(open('/tmp/stand_probe_v2.py').read().split('x=least_squares')[0])
from scipy.linalg import solve_discrete_are
z=np.load('/tmp/stand_eq.npz');d.qpos[:]=z['q'];d.ctrl[:]=z['u'];mj.mj_forward(m,d)
A=np.zeros((40,40));B=np.zeros((40,6));mj.mjd_transitionFD(m,d,1e-6,True,A,B,None,None)
Q=np.diag(np.r_[[1000,0,3000,2000,4000,100],[100 if 'wheel' not in m.joint(j).name else 0 for j in range(1,m.njnt)],[10,0,30,10,20,10],[1 if 'wheel' not in m.joint(j).name else .01 for j in range(1,m.njnt)]])
R=np.eye(6)*.1
P=Q.copy()
for i in range(10000):
 K=np.linalg.solve(R+B.T@P@B,B.T@P@A);Pn=Q+A.T@P@(A-B@K)
 if np.max(abs(Pn-P))<1e-6: break
 P=(Pn+Pn.T)/2
print('riccati',i,'eig',sorted(abs(np.linalg.eigvals(A-B@K)))[-8:]);print('K',K)
np.savez('/tmp/stand_lqr.npz',q=z['q'],u=z['u'],K=K,A=A,B=B,Q=Q,R=R)
d.qpos[:]=z['q'];d.qvel[:]=0
# perturb initial pitch .03 rad -- initialization only
rot=np.zeros(20);rot[4]=.03;mj.mj_integratePos(m,d.qpos,rot,1)
err=np.zeros(20)
for step in range(30000):
 mj.mj_differentiatePos(m,err,1,z['q'],d.qpos)
 d.ctrl[:]=np.clip(z['u']-K@np.r_[err,d.qvel],m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);mj.mj_step(m,d)
 if step%1000==0:print(step/1000,d.qpos[:7],np.max(abs(d.ctrl)), 'closure',max(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b]) for a,b in pairs),flush=True)
 if abs(d.qpos[5])>.3:break
