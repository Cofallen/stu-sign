"""Read-only dependency/import smoke checks; does not launch Kit or install packages."""
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
CHECKS = {
    "mjlab": [
        "/home/cofallen/miniconda3/envs/mjlab/bin/python", "-B", "-c",
        "import sys,importlib.metadata as m; import mujoco,mjlab; "
        "print(sys.version); print('MuJoCo import OK',mujoco.__version__,mujoco.__file__); "
        "print('mjlab import OK',m.version('mjlab'),mjlab.__file__); "
        "model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body pos=\"0 0 1\">"
        "<freejoint/><geom type=\"sphere\" size=\"0.1\" mass=\"1\"/></body></worldbody></mujoco>'); "
        "data=mujoco.MjData(model); mujoco.mj_step(model,data); print('Synthetic CPU step OK',data.time)",
    ],
    "isaac": [
        "/home/cofallen/.local/bin/isaac-python", "-B", "-c",
        "import sys,os,importlib.metadata as m; import isaacsim; from isaacsim import SimulationApp; "
        "print(sys.version); print('Isaac Sim import OK',m.version('isaacsim'),isaacsim.__file__); "
        "print('SimulationApp import OK',SimulationApp); print('ISAAC_PATH',os.environ.get('ISAAC_PATH')); "
        "print('No SimulationApp instance created')",
    ],
}


def main():
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'runs/m00_environment' / f'imports_{stamp}'
    output.mkdir(parents=True)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
               MPLCONFIGDIR=str(output / 'matplotlib'))
    results = {}
    for name, command in CHECKS.items():
        completed = subprocess.run(command, env=env, text=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, timeout=60)
        (output / f'{name}.log').write_text(completed.stdout)
        results[name] = {'command': command, 'returncode': completed.returncode}
        print(completed.stdout, end='')
        # Inventory snapshot only: never feed this automatically into pip install.
        python = command[0] if name == 'mjlab' else '/home/cofallen/miniconda3/envs/isaac/bin/python'
        snapshot = subprocess.run([python, '-B', '-c',
            "import importlib.metadata as m,json; print(json.dumps(sorted([(d.metadata['Name'],d.version) for d in m.distributions()]),indent=2))"],
            env=env, text=True, capture_output=True, timeout=60)
        (output / f'{name}_packages.json').write_text(snapshot.stdout)
        results[name]['inventory_returncode'] = snapshot.returncode
    (output / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))
    print('Evidence:', output)
    return int(any(r['returncode'] != 0 or r['inventory_returncode'] != 0 for r in results.values()))


if __name__ == '__main__':
    sys.exit(main())
