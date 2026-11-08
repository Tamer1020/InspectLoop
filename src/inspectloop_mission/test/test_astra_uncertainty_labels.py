"""Astra F6: no unsupported confidence/coverage claims in mission outputs."""
import json
import re

from inspectloop_mission import report

RESULT = {'status': 'ACCEPTED', 'value': 6.79, 'sigma_heuristic': 0.075,
          'band_2sigma_heuristic': [6.64, 6.94], 'alarm': 'NORMAL', 'observations': 2,
          'viewpoints_used': 2}


def _record():
    return {'mission_id': 'u1', 'mission_name': 'labels', 'state': 'COMPLETED', 'parameters': {},
            'gauges': [{'id': 'PG-12', 'unit': 'bar', 'viewpoints': [], 'timeline': [],
                        'result': dict(RESULT)}], 'health_events': []}


def test_reports_make_no_95_percent_or_confidence_claims(tmp_path):
    paths = report.write_reports(_record(), str(tmp_path))
    for k in ('markdown', 'html', 'json'):
        text = open(paths[k]).read()
        assert '95' not in re.sub(r'\d+\.\d+', '', text), f'{k} mentions 95'
        assert 'confidence interval' not in text.lower().replace('not a calibrated confidence '
                                                                 'interval', '')
    md = open(paths['markdown']).read()
    assert 'heuristic' in md.lower()


def test_result_text_labels_sigma_as_heuristic():
    t = report.result_text({'id': 'x', 'unit': 'bar', 'result': dict(RESULT)})
    assert 'heuristic' in t and '95' not in t


def test_legacy_records_with_sigma_key_still_render():
    legacy = dict(RESULT)
    legacy.pop('sigma_heuristic')
    legacy['sigma'] = 0.07
    t = report.result_text({'id': 'x', 'unit': 'bar', 'result': legacy})
    assert '6.79' in t and '95' not in t


def test_json_report_carries_uncertainty_note(tmp_path):
    rec = _record()
    paths = report.write_reports(rec, str(tmp_path))
    data = json.load(open(paths['json']))
    assert 'heuristic' in data['summary']['uncertainty_note'].lower()
