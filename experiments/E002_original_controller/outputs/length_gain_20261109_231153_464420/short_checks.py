from pathlib import Path
import subprocess,sys,json
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent/'short_checks';out.mkdir(exist_ok=False);results={}
for label,args in [('pitch_plus',['--case','pitch-plus']),('gas',['--variant','gas']),('mpc_compat',['--control','lqr-mpc'])]:
 command=[sys.executable,'-B',str(root/'controller.py'),'--headless','--duration','10',*args]
 with (out/(label+'.log')).open('w') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
 results[label]=dict(command=command,exit_code=result.returncode);print(label,result.returncode,flush=True)
(out/'results.json').write_text(json.dumps(results,indent=2));(out/'exit_status.json').write_text(json.dumps(dict(exit_code=int(any(v['exit_code'] for v in results.values())))))
