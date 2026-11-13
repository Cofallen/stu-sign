import numpy as np
from scipy.linalg import solve_discrete_are
z=np.load('/tmp/stand_lqr.npz');idx=np.array([i for i in range(40) if i not in [1,8,11]])
A=z['A'][np.ix_(idx,idx)];B=z['B'][idx];Q=z['Q'][np.ix_(idx,idx)];R=z['R']
try:
 P=solve_discrete_are(A,B,Q,R);K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
 print('maxeig',max(abs(np.linalg.eigvals(A-B@K))),'residual',np.max(abs(P-(Q+A.T@P@(A-B@K)))));np.savez('/tmp/stand_reduced.npz',K=K,A=A,B=B,Q=Q,R=R,P=P,idx=idx)
except Exception as e:print(type(e).__name__,e)
