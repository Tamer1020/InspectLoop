"""Offline evaluation of a gauge mission against the simulator's ground truth.

The mission and vision nodes never read the world layout; this tool does, *after* the run:

    ros2 run inspectloop_mission evaluate_run <run_dir> [--layout warehouse_layout.yaml]

For every gauge it reports the true value, the accepted value, the error and whether the error
lies within +-2 heuristic sigma, and - for every individual observation - the error, so rejected
observations can be checked too. All rates are reported with their denominators; with a handful
of gauges they are anecdotal, not statistics (sigma is a heuristic, see report.UNCERTAINTY_NOTE).
"""
import argparse
import json
import os
import sys
from typing import Dict

import yaml

INCORRECT_TOL = 0.10        # bar (1 % of full scale): accepted reading this far off = incorrect


def truth_table(layout: Dict) -> Dict[str, float]:
    return {g['id']: float(g['value']) for g in layout['gauges']['items']}


def evaluate(record: Dict, layout: Dict) -> Dict:
    truth = truth_table(layout)
    rows, obs_rows = [], []
    for g in record.get('gauges', []):
        t = truth.get(g['id'])
        r = g.get('result') or {}
        row = {'gauge': g['id'], 'truth': t, 'status': r.get('status')}
        if r.get('status') == 'ACCEPTED' and t is not None:
            err = r['value'] - t
            sig = r.get('sigma_heuristic', r.get('sigma'))
            row.update(value=r['value'], sigma=sig, error=round(err, 4),
                       abs_error=round(abs(err), 4),
                       within_2sigma=abs(err) <= 2 * sig,
                       incorrect=abs(err) > INCORRECT_TOL)
        rows.append(row)
        for v in g.get('viewpoints', []):
            for o in v.get('observations', []):
                e = None if o.get('value') is None or t is None else round(o['value'] - t, 4)
                obs_rows.append({'gauge': g['id'], 'index': o.get('index'),
                                 'viewpoint': v['viewpoint']['label'], 'value': o.get('value'),
                                 'sigma': o.get('sigma'), 'error': e,
                                 'flags': o.get('quality_flags'), 'decision': o.get('decision')})
    acc = [r for r in rows if r['status'] == 'ACCEPTED' and 'error' in r]
    rejected_with_value = [o for o in obs_rows if o['decision'] != 'ACCEPT' and o['error'] is not None]
    return {
        'mission_id': record.get('mission_id'),
        'accepted': len(acc),
        'unresolved': sum(1 for r in rows if r['status'] == 'UNRESOLVED'),
        'max_abs_error': max((r['abs_error'] for r in acc), default=None),
        'mean_abs_error': (round(sum(r['abs_error'] for r in acc) / len(acc), 4) if acc else None),
        'within_2sigma': sum(1 for r in acc if r['within_2sigma']),
        'incorrect_acceptances': sum(1 for r in acc if r['incorrect']),
        'incorrect_tolerance_bar': INCORRECT_TOL,
        'gauges': len(rows),
        'observations_total': len(obs_rows),
        'observations_accepted': sum(1 for o in obs_rows if o['decision'] == 'ACCEPT'),
        'rejected_observations_with_value': len(rejected_with_value),
        'rows': rows, 'observations': obs_rows,
    }


def default_layout() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('inspectloop_gazebo'), 'config',
                        'warehouse_layout.yaml')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--layout', default=None)
    args = ap.parse_args(argv)
    with open(os.path.join(args.run_dir, 'mission_record.json')) as f:
        record = json.load(f)
    with open(args.layout or default_layout()) as f:
        layout = yaml.safe_load(f)
    res = evaluate(record, layout)
    with open(os.path.join(args.run_dir, 'evaluation.json'), 'w') as f:
        json.dump(res, f, indent=2)
    print(f"{'gauge':<7} {'truth':>6} {'status':<11} {'value':>7} {'2sig_h':>7} {'error':>7} in2s")
    for r in res['rows']:
        if 'error' in r:
            print(f"{r['gauge']:<7} {r['truth']:>6.2f} {r['status']:<11} {r['value']:>7.3f} "
                  f"{2 * r['sigma']:>7.3f} {r['error']:>+7.3f} {'yes' if r['within_2sigma'] else 'NO'}")
        else:
            print(f"{r['gauge']:<7} {r['truth']:>6.2f} {r['status'] or '-':<11}")
    print('per-observation:')
    for o in res['observations']:
        err = '-' if o['error'] is None else f"{o['error']:+.3f}"
        print(f"  {o['gauge']} #{o['index']} {o['viewpoint']:<22} value={o['value']} err={err} "
              f"flags={o['flags']} -> {o['decision']}")
    print(f"accepted {res['accepted']}/{res['gauges']} gauges, unresolved {res['unresolved']}; "
          f"accepted observations {res['observations_accepted']}/{res['observations_total']}; "
          f"mean |err| {res['mean_abs_error']}, max |err| {res['max_abs_error']}; "
          f"within +-2 heuristic sigma {res['within_2sigma']}/{res['accepted']}; "
          f"incorrect acceptances (|err| > {INCORRECT_TOL} bar) "
          f"{res['incorrect_acceptances']}/{res['accepted']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
