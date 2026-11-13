from pathlib import Path
import sys,importlib.util,json,numpy as np,mujoco
root=Path(__file__).resolve().parents[2];r=Path(__file__).parent;sys.path.insert(0,str(root))
from LQR_six.lqr import GainSchedule,get_k,polyfit_matrix
spec=importlib.util.spec_from_file_location('old_gain',r/'before/LQR_six/lqr.py');old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
m=mujoco.MjModel.from_xml_path(str(root/'assets/robot_no_gas_spring.xml'));pt=json.loads((root/'outputs/length_gain_20261109_231153_464420/static_points.json').read_text())[5]
a=old.get_k(pt['length'],pt['qpos'],pt['ctrl'],m);b=get_k(pt['length'],pt['qpos'],pt['ctrl'],m)
xs=np.linspace(-1,1,9);matrices=np.array([np.eye(2)*x+x*x for x in xs]);fit_old=old.polyfit_matrix(xs,matrices,3);fit_new=polyfit_matrix(xs,matrices,3)
result=dict(gain_file_unchanged=(root/'gain_schedule.json').read_bytes()==(r/'before/gain_schedule.json').read_bytes(),linearization_max_difference=float(max(abs(x-y).max() for x,y in zip(a[:3],b[:3]))),polynomial_fit_max_difference=float(abs(fit_old-fit_new).max()))
result['passed']=result['gain_file_unchanged'] and result['linearization_max_difference']<1e-10 and result['polynomial_fit_max_difference']<1e-12
(r/'readability_audit.json').write_text(json.dumps(result,indent=2));print(result)
