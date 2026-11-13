exec(open('/tmp/stand_lqr_probe.py').read().split('A=np.zeros')[0])
zr=np.load('/tmp/stand_reduced.npz');idx=zr['idx'];Aa=zr['A'];Ba=zr['B'];Q=zr['Q'];R=zr['R'];A=np.linalg.matrix_power(Aa,10);B=sum(np.linalg.matrix_power(Aa,i)@Ba for i in range(10));P=solve_discrete_are(A,B,Q,R);K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
print('eig',max(abs(np.linalg.eigvals(A-B@K))))
rot=np.zeros(20);rot[4]=.03;mj.mj_integratePos(m,d.qpos,rot,1);err=np.zeros(20)
for step in range(10000):
 if step%10==0:
  mj.mj_differentiatePos(m,err,1,z['q'],d.qpos);d.ctrl[:]=np.clip(z['u']-K@np.r_[err,d.qvel][idx],m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
 mj.mj_step(m,d)
 if step%1000==0:print(step/1000,d.qpos[:7],max(abs(d.ctrl)),flush=True)
