import sys,json,numpy as np,mujoco
from pathlib import Path
p=Path(__file__).resolve().parents[2];sys.path.insert(0,str(p));from LQR_six.lqr import get_k
r=Path(__file__).parent;pts=json.loads((r/'static_points.json').read_text())+json.loads((r/'static_points_no-gas_holdout.json').read_text());m=mujoco.MjModel.from_xml_path(str(p/'assets/robot_no_gas_spring.xml'));res=[]
for L in [.28,.32,.33,.34]:
 point=min(pts,key=lambda a:abs(a['length']-L))
 for eps in [1e-5,1e-4,1e-3]:
  A,B,K,e=get_k(point['length'],point['qpos'],point['ctrl'],m,epsilon=eps);res.append(dict(L=L,epsilon=eps,K=K.tolist()));print(L,eps,K[1,4],K[0,1],flush=True)
(r/'epsilon_diagnostic.json').write_text(json.dumps(res,indent=2))
