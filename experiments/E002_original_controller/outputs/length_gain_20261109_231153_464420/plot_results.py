from pathlib import Path
import sys,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
r=Path(__file__).parent;root=r.parents[1];sys.path.insert(0,str(root));from LQR_six.lqr import GainSchedule
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');g=GainSchedule();L=np.linspace(.18,.34,200);K=np.array([g.evaluate(x)[2] for x in L]);samples=g.data
fig,axes=plt.subplots(2,2,figsize=(11,7),layout='constrained')
for row,col,label in [(0,0,'K[0,0]'),(0,4,'K[0,4]'),(1,0,'K[1,0]')]:
 ax=axes[0,0];ax.plot(L,K[:,row,col],label=label);ax.scatter(samples['lengths_m'],np.array(samples['samples']['K'])[:,row,col],s=13)
axes[0,0].set_title('腿长与部分 K 系数（点：直接求解；线：三次拟合）',fontproperties=font);axes[0,0].set_xlabel('L (m)');axes[0,0].legend()
for name,label in [('no_tuck','不收腿'),('tuck','腾空收腿')]:
 a=np.load(r/'jump_ablation'/(name+'.npz'))['samples'];mask=(a[:,0]>=2)&(a[:,0]<=3.8);a=a[mask]
 for ax,col,title in [(axes[0,1],12,'轮底离地间隙'),(axes[1,0],11,'整机质心高度'),(axes[1,1],7,'实际左腿长')]:
  ax.plot(a[:,0]-2,a[:,col],label=label);ax.set_title(title,fontproperties=font);ax.set_xlabel('time after J (s)');ax.set_ylabel('m');ax.legend(prop=font)
for ax in axes.flat:ax.grid(alpha=.25)
fig.savefig(r/'gain_and_jump.png',dpi=150);plt.close(fig)
