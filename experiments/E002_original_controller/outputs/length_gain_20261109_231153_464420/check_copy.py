from pathlib import Path
import shutil,tempfile,subprocess,sys,json,hashlib
r=Path(__file__).parent;root=r.parents[1];dest=Path(tempfile.mkdtemp(prefix='e002_length_copy_',dir='/tmp'))
for name in ['controller.py','gain_schedule.json','standing_pose.json','requirements.txt','README.md','THIRD_PARTY.md','SOURCE_MANIFEST.json']:
 shutil.copy(root/name,dest/name)
for name in ['LQR_six','assets']:shutil.copytree(root/name,dest/name,ignore=shutil.ignore_patterns('__pycache__'))
before=set((root/'outputs').iterdir());command=[sys.executable,'-B',str(dest/'controller.py'),'--headless','--duration','10']
with (r/'copy_console.log').open('w') as log:proc=subprocess.run(command,cwd='/tmp',stdout=log,stderr=subprocess.STDOUT)
run=next((dest/'outputs').iterdir());meta=json.loads((run/'metadata.json').read_text());checks=dict(exit_ok=proc.returncode==0,output_local=str(run).startswith(str(dest)),modules_local=all(str(p).startswith(str(dest)) for p in meta['runtime_paths']['modules'].values()),original_outputs_unchanged=before==set((root/'outputs').iterdir()))
(r/'copy_check.json').write_text(json.dumps(dict(command=command,copy_path=str(dest),run=str(run),checks=checks,passed=all(checks.values())),indent=2));print(checks);sys.exit(0 if all(checks.values()) else 1)
