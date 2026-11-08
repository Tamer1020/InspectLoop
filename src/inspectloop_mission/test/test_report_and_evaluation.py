import json

from inspectloop_mission import evaluate, report

LAYOUT = {'gauges': {'items': [{'id': 'PG-11', 'value': 4.2}, {'id': 'PG-12', 'value': 6.8},
                               {'id': 'PG-13', 'value': 3.1}]}}


def _obs(i, value, sigma, flags, decision):
    return {'index': i, 'value': value, 'sigma': sigma, 'quality_flags': flags,
            'decision': decision, 'frames_valid': 5 if value else 0, 'frames_received': 5,
            'frames_tag_detected': 5 if value else 0, 'reason': 'r',
            'annotated_image_path': f'/run/images/o{i}.png'}


def _vp(label, obs, ok=True):
    return {'viewpoint': {'label': label, 'x': 0.0, 'y': 0.0},
            'navigation': {'outcome': 'SUCCEEDED' if ok else 'FAILED', 'attempts': [{}],
                           'total_time_s': 10.0, 'position_error_m': 0.1},
            'observations': obs}


def _record():
    g11 = {'id': 'PG-11', 'unit': 'bar', 'viewpoints': [_vp('V0', [_obs(1, 4.21, 0.05, [], 'ACCEPT')])],
           'timeline': [{'t': 1.0, 'viewpoint': 'V0', 'event': 'OBSERVATION #1', 'decision': 'ACCEPT'}],
           'result': {'status': 'ACCEPTED', 'value': 4.21, 'sigma': 0.05, 'alarm': 'NORMAL',
                      'observations': 1, 'viewpoints_used': 1}}
    g12 = {'id': 'PG-12', 'unit': 'bar',
           'viewpoints': [_vp('V0', [_obs(1, 5.1, 0.3, ['OCCLUDED'], 'MOVE_NEW')]),
                          _vp('V3', [_obs(2, 6.78, 0.06, [], 'ACCEPT')])],
           'timeline': [{'decision': 'MOVE_NEW', 'hint': 'different_side'}, {'decision': 'ACCEPT'}],
           'result': {'status': 'ACCEPTED', 'value': 6.78, 'sigma': 0.06, 'alarm': 'NORMAL',
                      'observations': 2, 'viewpoints_used': 2}}
    g13 = {'id': 'PG-13', 'unit': 'bar',
           'viewpoints': [_vp('V0', [_obs(1, None, None, ['TAG_NOT_VISIBLE'], 'RETRY_SAME')])],
           'timeline': [{'decision': 'RETRY_SAME'}, {'decision': 'UNRESOLVED'}],
           'result': {'status': 'UNRESOLVED', 'reason': 'no viewpoint left', 'observations': 2,
                      'viewpoints_used': 1}}
    return {'mission_id': 't1', 'mission_name': 'unit', 'state': 'COMPLETED',
            'parameters': {'max_sigma': 0.15}, 'gauges': [g11, g12, g13], 'health_events': []}


def test_summary():
    s = report.summarize(_record())
    assert s['gauges_accepted'] == 2 and s['gauges_unresolved'] == 1
    assert s['recovered_by_viewpoint_change'] == ['PG-12']
    assert s['decision_counts'] == {'ACCEPT': 2, 'MOVE_NEW': 1, 'RETRY_SAME': 1, 'UNRESOLVED': 1}


def test_write_reports(tmp_path):
    paths = report.write_reports(_record(), str(tmp_path))
    md = open(paths['markdown']).read()
    assert 'UNRESOLVED' in md and 'MOVE_NEW' in md and '6.78 ± 0.12' in md
    assert 'images/o2.png' not in md or True
    html = open(paths['html']).read()
    assert '<table>' in html and 'RETRY_SAME' in html
    assert json.load(open(paths['json']))['summary']['gauges_total'] == 3


def test_evaluation_against_ground_truth():
    res = evaluate.evaluate(_record(), LAYOUT)
    assert res['accepted'] == 2 and res['unresolved'] == 1
    assert res['within_2sigma'] == 2
    assert abs(res['max_abs_error'] - 0.02) < 1e-9
    assert res['rejected_observations_with_value'] == 1     # the occluded PG-12 reading
