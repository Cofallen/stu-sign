from pathlib import Path
import datetime,json,hashlib,shutil,subprocess,sys
from zoneinfo import ZoneInfo
root=Path('/home/cofallen/Code/毕业设计');p=root/'experiments/E002_original_controller';stamp=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f');out=p/'outputs'/('copy_check_'+stamp);out.mkdir()
shutil.copy2(__file__,out/'check_author_copy.py')
sha=lambda x:hashlib.sha256(x.read_bytes()).hexdigest()
old=json.loads((root/'experiments/E001_parallel_standing/SOURCE_MANIFEST.json').read_text())['original_protected_sha256'];before={k:sha(root/k) for k in old}
copy=Path('/tmp')/('author_standing_'+stamp)/p.name;shutil.copytree(p,copy,ignore=shutil.ignore_patterns('outputs','__pycache__'))
cmd=[sys.executable,'-B',str(copy/'controller.py')]
with (out/'run.log').open('w') as log:r=subprocess.run(cmd,cwd='/tmp',stdout=log,stderr=subprocess.STDOUT)
run=next(x for x in (copy/'outputs').iterdir() if x.is_dir());metrics=json.loads((run/'results.json').read_text());meta=json.loads((run/'metadata.json').read_text());paths=meta['runtime_paths']
inside=all(Path(v).is_relative_to(copy) for v in [paths['project'],paths['output'],*paths['modules'].values()])
unchanged=before=={k:sha(root/k) for k in old}==old
result=dict(passed=r.returncode==0 and metrics['passed'] and inside and unchanged,command=cmd,cwd='/tmp',copy=str(copy),exit_code=r.returncode,metrics=metrics,runtime_paths=paths,all_runtime_modules_and_outputs_inside_copy=inside,old_protected_files_unchanged=unchanged,protected_file_count=len(old),symlinks=[str(x) for x in copy.rglob('*') if x.is_symlink()],limitations='Uses existing runtime; no exhaustive native filesystem trace, GUI/hardware or other-machine verification')
(out/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,'-B',__file__],cwd=str(Path.cwd()),python=sys.executable),indent=2)+'\n');(out/'exit_status.json').write_text(json.dumps(dict(exit_code=0 if result['passed'] else 1))+'\n');print(out);print(result['passed']);sys.exit(0 if result['passed'] else 1)
