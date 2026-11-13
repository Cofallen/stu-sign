import os
os.environ['MUJOCO_GL']='egl'
from pathlib import Path
import sys,json,mujoco
from PIL import Image
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root))
from LQR_six.render_stand import load_play_scene,make_camera
m=load_play_scene(root/'assets/robot_no_gas_spring.xml',root/'scenes/training_terrain.xml');d=mujoco.MjData(m);d.qpos[:]=json.loads((root/'standing_pose.json').read_text())['no-gas']['qpos'];mujoco.mj_forward(m,d)
with mujoco.Renderer(m,height=480,width=640) as render:
 render.update_scene(d,camera=make_camera(m,'terrain'));Image.fromarray(render.render()).save(Path(__file__).parent/'terrain.png')
