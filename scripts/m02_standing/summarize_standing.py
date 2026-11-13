#!/usr/bin/env python3
"""Aggregate explicit standing runs and draw plots without repeating simulation."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs', nargs='+', type=Path)
    args = p.parse_args()
    stamp = dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out = ROOT/'runs/m02_standing'/f'report_{stamp}'
    out.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('MPLCONFIGDIR', str(out/'matplotlib_cache'))
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summary = []
    sources = {}
    for run in args.runs:
        metadata = json.loads((run/'metadata.json').read_text())
        results = json.loads((run/'results.json').read_text())
        exit_code = json.loads((run/'exit_status.json').read_text())['exit_code']
        for case in results['results']:
            summary.append(dict(run=str(run), controller=metadata['args']['controller'],
                                variant=metadata['args']['variant'], exit_code=exit_code, **case))
        sources[str(run)] = hashlib.sha256((run/'results.json').read_bytes()).hexdigest()
    passed = all(r['passed'] and r['exit_code'] == 0 for r in summary)
    (out/'summary.json').write_text(json.dumps(dict(passed=passed, cases=summary), indent=2, ensure_ascii=False)+'\n')
    (out/'metadata.json').write_text(json.dumps(dict(command=[sys.executable, '-B', *sys.argv], cwd=str(Path.cwd()),
        python=sys.executable, source_results_sha256=sources, versions=dict(numpy=np.__version__, matplotlib=matplotlib.__version__),
        purpose='Aggregate explicit cases only; no statistical robustness estimate',
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), indent=2, ensure_ascii=False)+'\n')
    (out/'summarize_standing.py').write_bytes(Path(__file__).read_bytes())
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for run in args.runs:
        meta = json.loads((run/'metadata.json').read_text())['args']
        label = f"{meta['controller'].upper()} / {meta['variant']}"
        push = run/'push_trace.npz'
        if push.exists():
            tr = np.load(push)['metrics']; t = tr[:, 0]
            axes[0, 0].plot(t, tr[:, 5]*180/np.pi, label=label)
            axes[0, 1].plot(t, tr[:, 1]*100, label=label)
            axes[1, 0].plot(t, tr[:, 7]*1e6, label=label)
        large = run/'pitch-large_trace.npz'
        if large.exists():
            tr = np.load(large); t = tr['metrics'][:, 0]
            axes[1, 1].plot(t, tr['ctrl'][:, 1], label='MPC left wheel')
            axes[1, 1].plot(t, tr['ctrl'][:, 3], label='MPC right wheel')
    axes[0, 0].set(title='Recovery after 10 N push for 0.2 s', ylabel='Body pitch (deg)', xlim=(4, 12))
    axes[0, 1].set(title='Position recovery after push', ylabel='World X (cm)', xlim=(4, 12))
    axes[1, 0].set(title='Closed-pin residual during push', ylabel='Maximum closure error (micrometers)', xlim=(4, 12))
    axes[1, 1].set(title='MPC: initial pitch +0.1 rad', ylabel='Wheel torque (Nm)', xlim=(0, 2))
    axes[1, 1].axhline(5, color='black', ls=':', lw=1); axes[1, 1].axhline(-5, color='black', ls=':', lw=1)
    for ax in axes.flat:
        ax.set_xlabel('Simulation time (s)'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.savefig(out/'standing_evaluation.png', dpi=170)
    plt.close(fig)
    table = ['| Control | Model | Case | Seconds | Max drift (cm) | Final tilt peak (rad) | Max pin error (µm) | Pass |',
             '|---|---|---|---:|---:|---:|---:|---|']
    for r in summary:
        table.append(f"| {r['controller'].upper()} | {r['variant']} | {r['case']} | {r['duration_s']:.0f} | {100*r['max_xy_drift_m']:.4f} | {r['final_roll_pitch_peak_rad']:.3g} | {1e6*r['max_closure_m']:.3f} | {'yes' if r['passed'] else 'no'} |")
    (out/'table.md').write_text('\n'.join(table)+'\n')
    (out/'run.log').write_text(f'Aggregated {len(summary)} cases. Passed: {passed}\n')
    (out/'exit_status.json').write_text(json.dumps(dict(exit_code=0 if passed else 1))+'\n')
    print(out); print(f'{len(summary)} specified cases; all passed={passed}')
    return 0 if passed else 1


if __name__ == '__main__': raise SystemExit(main())
