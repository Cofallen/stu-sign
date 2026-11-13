from pathlib import Path
import sys,shutil,tempfile,subprocess,json
r=Path(__file__).parent;root=r.parents[1];dest=Path(tempfile.mkdtemp(prefix='e002_stacked_',dir='/tmp'))
for file in ['controller.py','gain_schedule.json','standing_pose.json','requirements.txt']:
 shutil.copy(root/file,dest/file)
for directory in ['LQR_six','assets']:
 shutil.copytree(root/directory,dest/directory,ignore=shutil.ignore_patterns('__pycache__'))
before=set((root/'outputs').iterdir());command=[sys.executable,'-B',str(dest/'controller.py'),'--headless','--duration','10']
with (r/'copy.log').open('w') as log:process=subprocess.run(command,cwd='/tmp',stdout=log,stderr=subprocess.STDOUT)
run=next((dest/'outputs').iterdir());meta=json.loads((run/'metadata.json').read_text());checks=dict(exit_ok=process.returncode==0,modules_local=all(str(f).startswith(str(dest)) for f in meta['runtime_paths']['modules'].values()),output_local=str(run).startswith(str(dest)),original_outputs_unchanged=before==set((root/'outputs').iterdir()))
(r/'copy_check.json').write_text(json.dumps(dict(command=command,copy_path=str(dest),run=str(run),checks=checks,passed=all(checks.values())),indent=2));print(checks)
