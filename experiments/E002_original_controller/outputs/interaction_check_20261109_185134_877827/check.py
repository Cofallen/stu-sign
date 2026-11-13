from pathlib import Path
import sys,json,hashlib,datetime,contextlib,ast
from zoneinfo import ZoneInfo
import mujoco,numpy as np
root=Path('/home/cofallen/Code/毕业设计');p=root/'experiments/E002_original_controller';sys.path.insert(0,str(p))
from LQR_six.render_stand import load_play_scene,sync_driving_viewer
out=p/'outputs'/('interaction_check_'+datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
raw=mujoco.MjModel.from_xml_path(str(p/'assets/robot_no_gas_spring.xml'));scene=load_play_scene(p/'assets/robot_no_gas_spring.xml')
physical={n:bool(np.array_equal(getattr(raw,n),getattr(scene,n))) for n in ['body_mass','body_inertia','body_ipos','body_iquat','jnt_axis','jnt_pos','dof_frictionloss','dof_damping','actuator_ctrlrange','eq_data']}
collision=[]
for i in range(raw.ngeom):
 if raw.geom_contype[i] or raw.geom_conaffinity[i]:
  for attr in ['geom_type','geom_size','geom_pos','geom_quat','geom_friction','geom_contype','geom_conaffinity']:
   assert np.array_equal(getattr(raw,attr)[i],getattr(scene,attr)[i])
  collision.append(raw.geom(i).name)
assert all(physical.values())
assert all(scene.geom(f'{s}_wheel_Link_visual_0').type[0]==mujoco.mjtGeom.mjGEOM_MESH for s in ['l','r'])
class Display:
 def __init__(self):
  self.opt=mujoco.MjvOption();self.user_scn=mujoco.MjvScene()
 def lock(self):return contextlib.nullcontext()
 def sync(self):
  self.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME]=1
  self.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]=1
v=Display();flags=(v.opt.flags.copy(),v.user_scn.flags.copy());flags[0][mujoco.mjtVisFlag.mjVIS_JOINT]=0;flags[1][mujoco.mjtRndFlag.mjRND_WIREFRAME]=0
sync_driving_viewer(v,flags);assert v.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME]==0 and v.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]==0
run=p/'outputs/lqr_no-gas_nominal_20261109_184411_704809';r=json.loads((run/'results.json').read_text());tr=np.load(run/'trace.npz');data=tr['samples'];cols=tr['columns'].tolist();motion=np.load(run/'motion.npz')['samples'];before_jump=motion[(motion[:,0]>1)&(motion[:,0]<5)];height=data[(data[:,0]>8)&(data[:,0]<9),cols.index('left_length')]
metrics=dict(final_x_m=r['final_base_xy_m'][0],duration_s=r['duration_s'],wheel_angle_change_during_walk_rad=(before_jump[-1,1:3]-before_jump[0,1:3]).tolist(),actual_left_length_after_QQ_m=float(height.mean()),airborne_seconds=r['teleop']['jumps'][0]['airborne_s'],jump_rise_m=r['teleop']['jumps'][0]['max_height']-r['teleop']['jumps'][0]['start_height'])
base=p/'outputs/lqr_no-gas_nominal_20261109_145716_529489/trace.npz';now=p/'outputs/lqr_no-gas_nominal_20261109_184251_796302/trace.npz'
identical=bool(np.array_equal(np.load(base)['samples'],np.load(now)['samples']));assert identical
for f in [p/'controller.py',*list((p/'LQR_six').glob('*.py'))]:ast.parse(f.read_text())
manifest=json.loads((p/'assets/manifest.json').read_text());unchanged={n:hashlib.sha256((p/'assets'/n).read_bytes()).hexdigest()==d for n,d in manifest['xml_sha256'].items()};assert all(unchanged.values())
result=dict(passed=True,physical_arrays_unchanged=physical,collision_geoms_unchanged=collision,formal_xml_hashes_match=unchanged,source_wheel_mesh_display=True,render_flags_unit_check=True,render_flags_scope='Actual API objects with simulated sync toggles; not a fresh end-to-end viewer test after final flag-lock patch',headless_standing_trace_equal_to_before=identical,actual_default_window_metrics=metrics,mouse_drag_automation_passed=False)
(out/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n');(out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable,'-B',__file__],cwd=str(Path.cwd()),python=sys.executable,versions={'mujoco':mujoco.__version__,'numpy':np.__version__}),indent=2)+'\n');(out/'exit_status.json').write_text('{"exit_code":0}\n');(out/'check.py').write_text(Path(__file__).read_text());print(out);print(json.dumps(result,indent=2,ensure_ascii=False))
