#!/usr/bin/env python3
"""Check MPC optimality/inactive-constraint LQR equivalence and asset integrity."""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src'))
import mujoco
import numpy as np
from wheel_leg.control.standing import LinearDesign, StandingController


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--design-run', type=Path, required=True)
    args = p.parse_args()
    stamp = dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out = ROOT/'runs/m02_standing'/f'design_check_{stamp}'
    out.mkdir(parents=True, exist_ok=False)
    (out/'check_design.py').write_bytes(Path(__file__).read_bytes())
    source = ROOT/'src/wheel_leg/control/standing.py'
    (out/'standing.py').write_bytes(source.read_bytes())
    meta = dict(command=[sys.executable, '-B', *sys.argv], cwd=str(Path.cwd()), python=sys.executable,
                design_run=str(args.design_run), design_sha256=hashlib.sha256((args.design_run/'design.npz').read_bytes()).hexdigest(),
                implementation_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                versions=dict(mujoco=mujoco.__version__, numpy=np.__version__), random_seed=None,
                thresholds=dict(unconstrained_gain_error=1e-7, bound_tolerance=1e-9,
                                projected_gradient_inf_norm=1e-4),
                purpose='Static algorithm checks, no dynamic simulation, no new tuning')
    (out/'metadata.json').write_text(json.dumps(meta, indent=2, ensure_ascii=False)+'\n')
    code = 2
    try:
        data = np.load(args.design_run/'design.npz')
        design = LinearDesign(**{k: data[k] for k in data.files}, info={})
        model_path = ROOT/json.loads((args.design_run/'metadata.json').read_text())['model']
        model = mujoco.MjModel.from_xml_path(str(model_path))
        ctrl = StandingController(model, design, mode='mpc')
        errors = []
        # All 37 coordinate directions at a small amplitude; limits inactive.
        for x in np.eye(len(design.indices))*1e-7:
            actual = ctrl.command(x)
            errors.append(np.max(np.abs(actual-(design.uref-design.K@x))))
        x = np.zeros(len(design.indices))
        x[np.where(design.indices == 4)[0][0]] = .1
        actual = ctrl.command(x)
        u = ctrl.last_solution
        gradient = ctrl.H@u+ctrl.F@x
        projected = float(np.max(np.abs(u-np.clip(u-gradient, ctrl.bounds_low, ctrl.bounds_high))))
        bound_violation = float(max(0., np.max(ctrl.bounds_low-u), np.max(u-ctrl.bounds_high)))
        manifest = json.loads((ROOT/'assets/robots/infantry_parallel/manifest.json').read_text())
        hashes = {name: hashlib.sha256((model_path.parent/name).read_bytes()).hexdigest() == value
                  for name, value in manifest['xml_sha256'].items()}
        result = dict(unconstrained_gain_max_error=float(max(errors)), constrained_solves=ctrl.constrained_solves,
                      solver_failures=ctrl.solver_failures, max_solver_iterations=ctrl.max_iterations,
                      box_bound_violation=bound_violation, projected_gradient_inf_norm=projected,
                      constrained_first_torque_nm=actual.tolist(), formal_assets_match_manifest=hashes)
        result['passed'] = bool(max(errors) < 1e-7 and ctrl.constrained_solves == 1 and ctrl.solver_failures == 0
                                and bound_violation < 1e-9 and projected < 1e-4 and all(hashes.values()))
        code = 0 if result['passed'] else 1
        text = json.dumps(result, indent=2, ensure_ascii=False)+'\n'
        (out/'results.json').write_text(text)
        (out/'run.log').write_text(text)
        print(out); print(text)
    except Exception:
        (out/'run.log').write_text(traceback.format_exc())
        traceback.print_exc()
    (out/'exit_status.json').write_text(json.dumps(dict(exit_code=code))+'\n')
    return code


if __name__ == '__main__': raise SystemExit(main())
