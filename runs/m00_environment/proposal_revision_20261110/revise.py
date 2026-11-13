from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from pathlib import Path
import hashlib,json,sys
root=Path.cwd(); src=root/'轮腿机器人动力学自适应和可微世界模型研究课题申报表.docx'; out=root/'基于动力学自适应的可微世界模型与轮腿机器人控制研究_课题申报表_修订版.docx'
d=Document(src); t=d.tables[0]
texts={
3:[
'面向负载与轮地接触条件变化下的平衡及低速运动，研究动力学自适应与可微世界模型相结合的策略学习方法。',
'1）依托现有完整并联轮腿机器人及 LQR 控制基础，核对仿真与实物的传感器、执行器和时序接口，建立可测或可估计状态与实际控制指令的数据集。',
'2）利用近期观测与动作历史提取动力学上下文，构建动作条件可微世界模型，预测姿态、角速度、轮速等短期状态变化；研究网络权重固定、在线更新上下文的适应机制。',
'3）在条件世界模型中开展短视界展开，通过模型梯度优化控制策略；分析动力学适应对多步预测误差及策略表现的影响，完成仿真对照，并逐步开展实物验证。'],
4:[
'1）保留完整并联闭链机构，统一状态、动作、单位与采样时序。部署输入采用可测或可估计信号，仿真特权状态仅用于监督或分析。',
'2）比较固定模型、历史条件模型与所研究的上下文适应方法，并以现有 LQR 和 PPO 为控制对照；各组统一数据或交互预算、任务条件与评测口径。',
'3）按完整轨迹和物理配置划分数据，在留出负载、摩擦组合及噪声条件下，评价多步预测误差、变化后恢复时间、速度跟踪误差、跌倒次数与推理时延。正式对照前固定阈值、时长及重复次数。',
'4）按离线预测、影子运行、受控闭环逐步进行实物评测，保留异常与失败记录；2027 年 5 月前完成毕业设计说明书及可复验材料。'],
5:[
'将动力学上下文用于可微世界模型的条件预测，并检验其对短视界策略学习的作用。重点研究测量噪声与动力学变化共同存在时的上下文更新问题，通过预测与控制两层评测及消融分析验证效果。具体改进与贡献以文献比较和实验结果为依据。'],
6:[
'1）毕业设计说明书，包含问题定义、方法设计、对照实验与局限分析。',
'2）完整并联模型、数据处理、模型训练、策略学习及部署接口代码与运行说明，附版本和来源记录。',
'3）数据划分清单、实验配置、日志、评测图表和演示资料；实物结果按实际完成范围报告。'],
7:[
'预期形成动力学变化条件下轮腿机器人模型预测与策略学习的可复验研究案例，为评估在线上下文适应的收益、计算成本和适用范围提供依据。研究可加深对动力学建模、自动微分、策略优化及虚实差异的理解，并为现有控制平台的后续改进提供参考。']}
def font(r,size=10.5,bold=False):
 r.font.name='Times New Roman';r.font.size=Pt(size);r.font.bold=bold;r.font.color.rgb=RGBColor(0,0,0)
 rf=r._element.get_or_add_rPr().rFonts;rf.set(qn('w:eastAsia'),'宋体')
def setcell(c,paras,size=10.5,align=WD_ALIGN_PARAGRAPH.JUSTIFY):
 c.text=''
 c.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
 for i,s in enumerate(paras):
  p=c.paragraphs[0] if i==0 else c.add_paragraph()
  p.alignment=align
  pp=p.paragraph_format;pp.left_indent=Pt(0);pp.right_indent=Pt(0);pp.first_line_indent=Pt(0);pp.space_before=Pt(0);pp.space_after=Pt(2);pp.line_spacing=Pt(14)
  font(p.add_run(s),size)
for ri,paras in texts.items():setcell(t.cell(ri,1),paras)
setcell(t.cell(0,2),['基于动力学自适应的可微世界模型与轮腿机器人控制研究'],10.5,WD_ALIGN_PARAGRAPH.CENTER)
setcell(t.cell(2,5),['应用研究'],10.5,WD_ALIGN_PARAGRAPH.CENTER)
# Keep the supplied personnel and approval fields unchanged; normalize label wrapping.
for ri,label in [(3,'主要研究内容'),(4,'目标和要求'),(5,'特色'),(6,'成果形式'),(7,'成果价值')]:
 setcell(t.cell(ri,0),[label],10,WD_ALIGN_PARAGRAPH.CENTER)
for ri in range(3,8):
 row=t.rows[ri]; row.height=None
 pr=row._tr.get_or_add_trPr()
 e=OxmlElement('w:cantSplit');pr.append(e)
# Existing form design is retained; remove floating textboxes by replacing only research cells.
from docx.enum.style import WD_STYLE_TYPE
if 'Title' not in d.styles: d.styles.add_style('Title',WD_STYLE_TYPE.PARAGRAPH)
p=d.paragraphs[0];p.style=d.styles['Title'];p.alignment=WD_ALIGN_PARAGRAPH.CENTER
p.paragraph_format.left_indent=Pt(0);p.paragraph_format.right_indent=Pt(0);p.paragraph_format.space_after=Pt(0);p.paragraph_format.line_spacing=Pt(24)
for r in p.runs:font(r,16,True)
# Reduce inherited empty-paragraph spacing outside the form.
for p in d.paragraphs[1:]:
 if not p.text:
  p.paragraph_format.space_before=Pt(0);p.paragraph_format.space_after=Pt(0);p.paragraph_format.line_spacing=Pt(2)
  for r in p.runs:font(r,2)
d.save(out)
manifest={'source':str(src),'output':str(out),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'output_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'python':sys.executable,'changed_rows':[0,2,3,4,5,6,7],'approval_fields':'preserved exactly from source; not verified','status':'authored; rendering pending'}
(root/'runs/m00_environment/proposal_revision_20261110/manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
print(out)
print('characters',sum(len(s) for a in texts.values() for s in a))
