from pathlib import Path
import tempfile,shutil,subprocess,sys,json
r=Path(__file__).parent;root=r.parents[1];dest=Path(tempfile.mkdtemp(prefix='e002_terrain_',dir='/tmp'));before=set((root/'outputs').iterdir())
for n in ('controller.py','standing_pose.json','gain_schedule.json','requirements.txt'):shutil.copy(root/n,dest/n)
for n in ('LQR_six','assets','scenes'):shutil.copytree(root/n,dest/n,ignore=shutil.ignore_patterns('__pycache__'))
command=[sys.executable,'-B',str(dest/'controller.py'),'--headless','--terrain','training','--duration','10']
with (r/'copy_terrain.log').open('w') as f:code=subprocess.run(command,cwd='/tmp',stdout=f,stderr=subprocess.STDOUT).returncode
run=next((dest/'outputs').iterdir());meta=json.loads((run/'metadata.json').read_text());checks=dict(exit_ok=code==0,local_modules=all(str(v).startswith(str(dest)) for v in meta['runtime_paths']['modules'].values()),local_outputs=str(run).startswith(str(dest)),original_outputs_unchanged=before==set((root/'outputs').iterdir()),scene_recorded='scenes/training_terrain.xml' in meta['file_sha256'])
(r/'copy_check.json').write_text(json.dumps(dict(command=command,copy_path=str(dest),checks=checks,passed=all(checks.values())),indent=2));print(checks)
