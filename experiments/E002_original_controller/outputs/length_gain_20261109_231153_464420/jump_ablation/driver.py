from pathlib import Path
import json,sys,shutil,numpy as np
from experiment import run,ROOT
out=Path(__file__).parent/'jump_ablation';out.mkdir(exist_ok=False)
results={}
for enabled in (False,True):
 name='tuck' if enabled else 'no_tuck';r,a,q,u,m=run('jump',seconds=10,tuck=enabled);results[name]=r;np.savez_compressed(out/(name+'.npz'),samples=a,qpos=q,ctrl=u);print(name,r['failed'],r['max_base_z'],r['max_com_z'],r['max_wheel_clearance'],flush=True)
(out/'results.json').write_text(json.dumps(results,indent=2));(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,*sys.argv],seed=None,model='no-gas',physics_dt=.001,control_dt=.01,comparison='identical fitted gains and thrust; only tuck_enabled differs'),indent=2));(out/'exit_status.json').write_text(json.dumps(dict(exit_code=int(any(r['failed'] for r in results.values())))));shutil.copy(__file__,out/'driver.py')
