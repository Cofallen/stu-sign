from pathlib import Path
import sys,json,numpy as np,mujoco
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root))
from LQR_six.lqr import GainSchedule,get_k
p=Path(__file__).parent;m=mujoco.MjModel.from_xml_path(str(root/'assets/robot_no_gas_spring.xml'));g=GainSchedule();rows=[]
for point in json.loads((p/'static_points_no-gas_holdout.json').read_text()):
 A,B,K,error=get_k(point['length'],point['qpos'],point['ctrl'],m);Af,Bf,Kf,L=g.evaluate(point['length']);row=dict(length=L,max_closed_loop_real=float(np.linalg.eigvals(A-B@Kf).real.max()),relative_K_error=float(np.linalg.norm(Kf-K)/np.linalg.norm(K)),max_K_entry_error=float(abs(Kf-K).max()),tangent_error=error);rows.append(row);print(row,flush=True)
r=dict(command=[sys.executable,*sys.argv],holdout=rows,passed=all(r['max_closed_loop_real']<0 and r['relative_K_error']<.02 for r in rows))
(p/'holdout_audit.json').write_text(json.dumps(r,indent=2));sys.exit(0 if r['passed'] else 1)
