"""Download read-only research references; never install or execute upstream code."""
import concurrent.futures, datetime, hashlib, html, json, os, re, subprocess, sys
from pathlib import Path
from zoneinfo import ZoneInfo
ROOT=Path(__file__).resolve().parents[2]
CAT=ROOT/(sys.argv[1] if len(sys.argv)>1 else 'references/reading_catalog.json')
now=datetime.datetime.now(ZoneInfo('Asia/Shanghai'))
RUN=ROOT/'runs/m00_environment'/('literature_fetch_'+now.strftime('%Y%m%d_%H%M%S_%f'))
RUN.mkdir(parents=True,exist_ok=False)
CATALOG=json.loads(CAT.read_text())
env=dict(os.environ,GIT_LFS_SKIP_SMUDGE='1',GIT_TERMINAL_PROMPT='0')
def call(args,log,timeout=180):
    with log.open('a') as f:
        f.write(json.dumps(args,ensure_ascii=False)+'\n'); f.flush()
        r=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,env=env,timeout=timeout)
        f.write(f'\nexit_code={r.returncode}\n')
    if r.returncode: raise RuntimeError(f'exit {r.returncode}; see {log.relative_to(ROOT)}')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def fetch(item):
    out=dict(item,accessed=now.isoformat(),status='failed')
    log=RUN/(item['id']+'.log')
    try:
        dest=ROOT/item['local']; dest.parent.mkdir(parents=True,exist_ok=True)
        if dest.exists(): raise FileExistsError(f'refusing to overwrite {dest}')
        if item['kind']=='paper':
            meta=RUN/(item['id']+'.html')
            call(['curl','-fLsS','--retry','2','--connect-timeout','20','--max-time','90',item['url'],'-o',str(meta)],log)
            page=meta.read_text(errors='replace')
            def tags(name):
                return [html.unescape(x) for x in re.findall(r'<meta\s+name="'+name+r'"\s+content="([^"]*)"',page)]
            out['title']=next(iter(tags('citation_title')),item['title'])
            out['authors']=tags('citation_author')
            out['date']=next(iter(tags('citation_date')),None)
            versions=re.findall(re.escape(item['arxiv'])+r'v(\d+)',page)
            version='v'+str(max(map(int,versions))) if versions else None
            out['version']=version or 'unresolved'
            pdfurl='https://arxiv.org/pdf/'+item['arxiv']+(version or '')
            out['pdf_url']=pdfurl
            part=RUN/(item['id']+'.pdf.part')
            call(['curl','-fLsS','--retry','2','--connect-timeout','20','--max-time','150',pdfurl,'-o',str(part)],log,200)
            if not part.read_bytes().startswith(b'%PDF-'): raise ValueError('not a PDF')
            call(['pdfinfo',str(part)],log)
            part.rename(dest)
            call(['pdftotext','-layout',str(dest),str(RUN/(item['id']+'.txt'))],log)
            out.update(sha256=sha(dest),bytes=dest.stat().st_size,status='downloaded_verified_pdf')
        else:
            call(['git','-c','core.hooksPath=/dev/null','clone','--depth','1','--no-recurse-submodules',item['url'],str(dest)],log,300)
            out['commit']=subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip()
            out['tree']=subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD^{tree}'],text=True).strip()
            out['tracked_status']=subprocess.check_output(['git','-C',str(dest),'status','--porcelain'],text=True).strip()
            licenses=[p for p in dest.iterdir() if p.is_file() and (p.name.lower().startswith(('license','copying','notice')))]
            out['license_files']=[dict(path=str(p.relative_to(ROOT)),sha256=sha(p)) for p in licenses]
            out['license']='pending text inspection' if licenses else 'unknown: no root license file'
            out['submodules_initialized']=False;out['lfs_downloaded']=False
            out['status']='downloaded_read_only_not_executed'
        out['exit_code']=0
    except Exception as e: out.update(error=str(e),exit_code=1)
    (RUN/(item['id']+'.json')).write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    print(item['id']+': '+out['status'],flush=True)
    return out
(RUN/'invocation.json').write_text(json.dumps(dict(command=[sys.executable,*sys.argv],cwd=str(Path.cwd()),interpreter=sys.executable,catalog_sha256=sha(CAT),seed=None,purpose='read-only reference acquisition; no training',started=now.isoformat()),ensure_ascii=False,indent=2)+'\n')
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
    results=list(pool.map(fetch,CATALOG['items']))
(RUN/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
print('RUN='+str(RUN),flush=True)
sys.exit(any(x['exit_code'] for x in results))
