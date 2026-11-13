from pathlib import Path
from docx import Document
from zipfile import ZipFile
import json,hashlib
p=Path('runs/m00_environment/proposal_revision_20261110');m=json.loads((p/'manifest.json').read_text());a=Document(m['source']);b=Document(m['output'])
assert hashlib.sha256(Path(m['source']).read_bytes()).hexdigest()==m['source_sha256']
for i in (8,9):assert a.tables[0].rows[i]._tr.xml==b.tables[0].rows[i]._tr.xml
for ri,ci in [(1,2),(1,4),(1,6),(2,2)]:assert a.tables[0].cell(ri,ci).text==b.tables[0].cell(ri,ci).text
with ZipFile(m['output']) as z:
 assert z.testzip() is None
 xml=z.read('word/document.xml').decode()
 assert 'txbxContent' not in xml
 assert '串联' not in xml and 'SuperMimic' not in xml
for ri in range(3,8):assert b.tables[0].cell(ri,1).text.strip()
m['checks']={'source_unchanged':True,'zip_integrity':True,'approval_rows_unchanged':True,'personnel_fields_unchanged':True,'floating_textboxes_removed':True,'research_cells_nonempty':True}
m['status']='content and OOXML checks passed; visual pagination unverified'
m['render']={'exit_code':1,'error':'Bundled LibreOffice requires GLIBC_2.38 and GLIBCXX_3.4.31 unavailable on host','log':'render.log','images_generated':0}
(p/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2));print(json.dumps(m,ensure_ascii=False,indent=2))
