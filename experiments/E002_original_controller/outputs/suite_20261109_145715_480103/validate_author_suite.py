from pathlib import Path
import datetime,json,subprocess,sys,shutil
from zoneinfo import ZoneInfo
p=Path('/home/cofallen/Code/毕业设计/experiments/E002_original_controller')
stamp=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f');out=p/'outputs'/('suite_'+stamp);out.mkdir()
shutil.copy2(__file__,out/'validate_author_suite.py')
checks=[('lqr','no-gas','nominal',10),('lqr','no-gas','pitch-minus',10),('lqr','no-gas','roll',10),('lqr','no-gas','push',30),('lqr','no-gas','pitch-plus',30),('lqr-mpc','no-gas','push',10),('lqr','gas','nominal',10),('lqr-mpc','gas','nominal',10)]
results=[]
for control,variant,case,duration in checks:
 cmd=[sys.executable,'-B',str(p/'controller.py'),'--control',control,'--variant',variant,'--case',case,'--duration',str(duration)]
 label=f'{control}_{variant}_{case}';before=set((p/'outputs').iterdir())
 with (out/(label+'.log')).open('w') as log:r=subprocess.run(cmd,cwd='/tmp',stdout=log,stderr=subprocess.STDOUT)
 new=[x for x in set((p/'outputs').iterdir())-before if x.is_dir()]
 if len(new)!=1:raise RuntimeError(f'Unexpected outputs {new}')
 data=json.loads((new[0]/'results.json').read_text());results.append(dict(command=cmd,exit_code=r.returncode,run=new[0].name,**data))
 (out/'results.json').write_text(json.dumps(dict(passed=all(x['passed'] and x['exit_code']==0 for x in results),cases=results),indent=2,ensure_ascii=False)+'\n')
 print(label,r.returncode,data['passed'],flush=True)
 if r.returncode:break
code=0 if len(results)==len(checks) and all(x['passed'] for x in results) else 1
(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,'-B',__file__],cwd=str(Path.cwd()),planned_checks=checks),ensure_ascii=False,indent=2)+'\n')
(out/'exit_status.json').write_text(json.dumps(dict(exit_code=code))+'\n');print(out,flush=True);sys.exit(code)
