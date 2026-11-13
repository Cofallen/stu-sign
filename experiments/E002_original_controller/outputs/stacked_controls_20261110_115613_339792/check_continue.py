"""通过真实主入口验证：施加测试倾斜后记录fall，但不由fall终止。"""
from pathlib import Path
import sys,numpy as np,json
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root));import controller
original=controller.mujoco.mj_step;injected=False

def step(model,data):
 global injected
 if not injected and data.time>=1.:
  offset=np.zeros(model.nv);offset[3]=.8
  controller.mujoco.mj_integratePos(model,data.qpos,offset,1.)
  data.qpos[2]+=.2  # 测试抬高后倾斜，避免瞬间穿透地面。
  controller.mujoco.mj_forward(model,data);injected=True
 return original(model,data)

controller.mujoco.mj_step=step
sys.argv=[str(root/'controller.py'),'--headless','--duration','10']
code=controller.main()
# 预期站立验收会判失败；本检查验证运行确实继续而非伪报站立成功。
print('standing_exit_code',code)
