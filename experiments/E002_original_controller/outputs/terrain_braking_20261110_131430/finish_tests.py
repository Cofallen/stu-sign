from pathlib import Path
import json,numpy as np,sys,subprocess
from experiment import run,ROOT
p=Path(__file__).parent/'response_v3';p.mkdir(exist_ok=False);res={}
for kind in ['move','spin_right','jump','move_jump']:
 r,a,q,u,m=run(kind,seconds=20 if kind=='spin_right' else 12)
 np.savez_compressed(p/(kind+'.npz'),samples=a,qpos=q,ctrl=u);res[kind]=r;print(kind,r['failed'],r['final_tilt'],flush=True)
 if kind=='move':
  speed=np.gradient(a[:,1],a[:,0]);stops=[]
  for t in [3.,7.]:
   ix=np.where((a[:,0]>=t)&(abs(speed)<.05))[0];stops.append(float(a[ix[0],0]-t) if len(ix) else None)
  r['release_to_body_speed_below_005_s']=stops
 if kind=='jump':r['last_length']=float(a[-1,7])
(p/'results.json').write_text(json.dumps(res,indent=2));(p/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,*sys.argv],seed=None,physics_dt=.001,source='experiment.py current E002 modules'),indent=2))
command=[sys.executable,'-B',str(ROOT/'controller.py'),'--headless','--terrain','training','--duration','10']
with (p/'terrain.log').open('w') as f:code=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT).returncode
(p/'terrain_exit.json').write_text(json.dumps(dict(command=command,exit_code=code)))
