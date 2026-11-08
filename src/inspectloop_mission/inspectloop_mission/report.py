"""Gauge inspection report generation from a mission record (ROS independent).

The mission manager writes `mission_record.json`; this module renders
`mission_report.json/.md/.html` next to it. It can be re-run offline:

    ros2 run inspectloop_mission generate_report <run_dir>
"""
import argparse
import html
import json
import os
import sys
from typing import Dict, List


def decisions_of(g: Dict) -> List[str]:
    return [e['decision'] for e in g.get('timeline', [])]


def summarize(record: Dict) -> Dict:
    gs = record.get('gauges', [])
    acc = [g for g in gs if (g.get('result') or {}).get('status') == 'ACCEPTED']
    unres = [g for g in gs if (g.get('result') or {}).get('status') == 'UNRESOLVED']
    counts: Dict[str, int] = {}
    for g in gs:
        for d in decisions_of(g):
            counts[d] = counts.get(d, 0) + 1
    navs = [v['navigation'] for g in gs for v in g.get('viewpoints', [])]
    errs = [n['position_error_m'] for n in navs
            if n.get('outcome') == 'SUCCEEDED' and n.get('position_error_m') is not None]
    return {
        'gauges_total': len(gs),
        'gauges_accepted': len(acc),
        'gauges_unresolved': len(unres),
        'alarms': [g['id'] for g in acc if g['result'].get('alarm') != 'NORMAL'],
        'recovered_by_viewpoint_change': [g['id'] for g in acc
                                          if g['result'].get('viewpoints_used', 1) > 1],
        'decision_counts': counts,
        'observations_total': sum((g.get('result') or {}).get('observations', 0) for g in gs),
        'viewpoints_total': len(navs),
        'navigation_succeeded': sum(1 for n in navs if n.get('outcome') == 'SUCCEEDED'),
        'mean_position_error_m': round(sum(errs) / len(errs), 3) if errs else None,
    }


def _rel(path: str, run_dir: str) -> str:
    if not path:
        return ''
    try:
        return os.path.relpath(path, run_dir)
    except ValueError:
        return path


def _f(v, nd=2):
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def result_text(g: Dict) -> str:
    r = g.get('result') or {}
    if r.get('status') == 'ACCEPTED':
        return (f"{r['value']:.2f} ± {2 * r['sigma']:.2f} {g.get('unit', '')} (95%), "
                f"alarm {r['alarm']}")
    if r.get('status') == 'UNRESOLVED':
        return f"UNRESOLVED — {r.get('reason', '')}"
    return 'not inspected'


def render_markdown(record: Dict, run_dir: str) -> str:
    s = summarize(record)
    L: List[str] = []
    L.append(f"# InspectLoop Gauge Inspection Report — {record.get('mission_name', '')}")
    L.append('')
    L.append(f"- **Mission ID:** `{record.get('mission_id')}`")
    L.append(f"- **Mission state:** **{record.get('state')}**"
             + (f" — {record['abort_reason']}" if record.get('abort_reason') else ''))
    L.append(f"- **Started (wall clock):** {record.get('started_wall')}")
    L.append(f"- **Duration:** {_f(record.get('duration_sim_s'), 1)} s sim time / "
             f"{_f(record.get('duration_wall_s'), 1)} s wall time")
    L.append('- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV '
             'from simulated camera frames captured after the robot reached the viewpoint; the '
             'robot does not know the true gauge values.')
    p = record.get('parameters', {})
    L.append(f"- **Acceptance rule:** sigma ≤ {p.get('max_sigma')} , ≥ {p.get('min_valid_frames')} "
             f"valid frames of {p.get('frames_per_observation')}, no blocking quality flag; "
             f"≤ {p.get('max_retries_same')} RETRY_SAME per viewpoint, ≤ {p.get('max_viewpoints')} "
             f"viewpoints and ≤ {p.get('max_observations')} observations per gauge.")
    L.append('')
    L.append('## Summary')
    L.append('')
    L.append(f"Gauges: **{s['gauges_total']}** — accepted **{s['gauges_accepted']}**, unresolved "
             f"**{s['gauges_unresolved']}**. Observations: {s['observations_total']}, viewpoints "
             f"visited: {s['viewpoints_total']} (navigation succeeded {s['navigation_succeeded']}, "
             f"mean final position error {_f(s['mean_position_error_m'], 3)} m).")
    L.append('')
    L.append('Decisions: ' + ', '.join(f'**{k}** ×{v}' for k, v in s['decision_counts'].items()))
    L.append('')
    if s['alarms']:
        L.append('**Alarm limits exceeded:** ' + ', '.join(s['alarms']))
        L.append('')
    if s['recovered_by_viewpoint_change']:
        L.append('**Recovered by autonomous viewpoint change:** '
                 + ', '.join(s['recovered_by_viewpoint_change']))
        L.append('')
    L.append('| Gauge | Result | Observations | Viewpoints | Decision sequence |')
    L.append('|---|---|---|---|---|')
    for g in record.get('gauges', []):
        r = g.get('result') or {}
        L.append(f"| {g['id']} | {result_text(g)} | {r.get('observations', '-')} | "
                 f"{r.get('viewpoints_used', '-')} | {' → '.join(decisions_of(g))} |")
    L.append('')
    for g in record.get('gauges', []):
        r = g.get('result') or {}
        L.append(f"## {g['id']} — {g.get('description', '')}")
        L.append('')
        L.append(f"**{result_text(g)}**  ")
        L.append(f"Feasible viewpoint candidates: {len(g.get('candidates_feasible', []))}, "
                 f"rejected by the map (collision/clearance): {len(g.get('candidates_rejected', []))}.")
        L.append('')
        L.append('| t (s) | Viewpoint | Event | Value | σ | Quality flags | Decision | Reason |')
        L.append('|---|---|---|---|---|---|---|---|')
        for e in g.get('timeline', []):
            L.append(f"| {_f(e.get('t'), 1)} | {e.get('viewpoint')} | {e.get('event')} | "
                     f"{_f(e.get('value'), 3)} | {_f(e.get('sigma'), 3)} | "
                     f"{', '.join(e.get('flags') or []) or '-'} | **{e.get('decision')}** | "
                     f"{e.get('reason', '')}{' (hint: ' + e['hint'] + ')' if e.get('hint') else ''} |")
        L.append('')
        for v in g.get('viewpoints', []):
            nav = v['navigation']
            L.append(f"- **{v['viewpoint']['label']}** at ({v['viewpoint']['x']}, "
                     f"{v['viewpoint']['y']}): navigation {nav['outcome']} in "
                     f"{_f(nav.get('total_time_s'), 1)} s, final error "
                     f"{_f(nav.get('position_error_m'), 3)} m")
            for o in v.get('observations', []):
                L.append(f"  - obs #{o.get('index')}: frames {o.get('frames_valid')}/"
                         f"{o.get('frames_received')} valid (tag in {o.get('frames_tag_detected')}), "
                         f"visibility {_f(o.get('dial_visibility'))}, needle conf "
                         f"{_f(o.get('needle_confidence'))}, view angle "
                         f"{_f(o.get('view_angle_deg'), 1)}°, distance {_f(o.get('distance_m'))} m, "
                         f"dial radius {_f(o.get('dial_radius_px'), 1)} px → {o.get('decision')}")
                if o.get('annotated_image_path'):
                    L.append('')
                    L.append(f"    ![{g['id']} obs {o.get('index')}]"
                             f"({_rel(o['annotated_image_path'], run_dir)})")
                    L.append('')
        L.append('')
    home = record.get('home_return')
    if home:
        L.append(f"## Return to home\n\n{home.get('outcome')} "
                 f"({len(home.get('attempts', []))} attempt(s)).\n")
    events = record.get('health_events', [])
    L.append('## System health events')
    L.append('')
    if events:
        L.append('| Sim time (s) | Component | Level | Message |')
        L.append('|---|---|---|---|')
        for e in events[:200]:
            L.append(f"| {_f(e.get('t'), 1)} | {e.get('name')} | {e.get('level')} | "
                     f"{e.get('message')} |")
    else:
        L.append('No health level changes were recorded during the mission.')
    L.append('')
    return '\n'.join(L)


_CSS = """body{font-family:system-ui,sans-serif;margin:24px;color:#1d2330;background:#fafbfc;max-width:1300px}
table{border-collapse:collapse;margin:10px 0}td,th{border:1px solid #ccd;padding:4px 8px;font-size:13px}
th{background:#eef1f5}.ACCEPT,.ACCEPTED{color:#18794e;font-weight:700}.RETRY_SAME{color:#a16207;font-weight:700}
.MOVE_NEW{color:#c2410c;font-weight:700}.UNRESOLVED{color:#b91c1c;font-weight:700}
.cards{display:flex;flex-wrap:wrap;gap:12px}.card{background:#fff;border:1px solid #dde;border-radius:8px;
padding:8px;width:400px;font-size:13px}.card img{width:100%;border-radius:4px}"""


def render_html(record: Dict, run_dir: str) -> str:
    s = summarize(record)
    e = html.escape
    rows, sections = [], []
    for g in record.get('gauges', []):
        r = g.get('result') or {}
        st = r.get('status', '-')
        rows.append(f"<tr><td>{e(g['id'])}</td><td class='{st}'>{e(result_text(g))}</td>"
                    f"<td>{r.get('observations', '-')}</td><td>{r.get('viewpoints_used', '-')}</td>"
                    f"<td>{' → '.join(f'<span class={d}>{d}</span>' for d in decisions_of(g))}</td></tr>")
        cards = []
        for v in g.get('viewpoints', []):
            for o in v.get('observations', []):
                img = o.get('annotated_image_path')
                val = (f"{o['value']:.2f} ± {o['sigma']:.2f}" if o.get('value') is not None
                       else 'no value')
                cards.append(
                    f"<div class='card'><b>#{o.get('index')} {e(v['viewpoint']['label'])}</b><br>"
                    f"{e(val)} · flags: {e(', '.join(o.get('quality_flags') or []) or 'none')}<br>"
                    f"<span class='{o.get('decision')}'>{o.get('decision')}</span> — "
                    f"{e(o.get('reason', ''))}"
                    + (f"<img src='{e(_rel(img, run_dir))}'>" if img else '') + '</div>')
            if v['navigation']['outcome'] != 'SUCCEEDED':
                cards.append(f"<div class='card'><b>{e(v['viewpoint']['label'])}</b><br>"
                             f"navigation {e(v['navigation']['outcome'])} - viewpoint unreachable</div>")
        sections.append(f"<h2>{e(g['id'])} — {e(g.get('description', ''))}</h2>"
                        f"<p class='{st}'>{e(result_text(g))}</p><div class='cards'>{''.join(cards)}</div>")
    dec = ', '.join(f'{k}: {v}' for k, v in s['decision_counts'].items())
    return f"""<!doctype html><html><head><meta charset="utf-8">
<title>InspectLoop gauge report {e(str(record.get('mission_id')))}</title><style>{_CSS}</style></head><body>
<h1>InspectLoop Gauge Inspection Report</h1>
<p><b>{e(str(record.get('mission_name')))}</b> · mission <code>{e(str(record.get('mission_id')))}</code> ·
state <b>{e(str(record.get('state')))}</b> {e(record.get('abort_reason') or '')}<br>
{_f(record.get('duration_sim_s'), 1)} s sim / {_f(record.get('duration_wall_s'), 1)} s wall ·
source: Gazebo Harmonic simulation, values read by OpenCV from simulated camera frames</p>
<p>Gauges {s['gauges_total']}: accepted {s['gauges_accepted']}, unresolved {s['gauges_unresolved']} ·
observations {s['observations_total']} · viewpoints {s['viewpoints_total']} · decisions: {e(dec)}</p>
<table><tr><th>Gauge</th><th>Result</th><th>Obs.</th><th>Viewpoints</th><th>Decisions</th></tr>
{''.join(rows)}</table>{''.join(sections)}</body></html>"""


def write_reports(record: Dict, run_dir: str) -> Dict[str, str]:
    record = dict(record)
    record['summary'] = summarize(record)
    paths = {k: os.path.join(run_dir, f'mission_report.{ext}')
             for k, ext in (('json', 'json'), ('markdown', 'md'), ('html', 'html'))}
    with open(paths['json'], 'w') as f:
        json.dump(record, f, indent=2)
    with open(paths['markdown'], 'w') as f:
        f.write(render_markdown(record, run_dir))
    with open(paths['html'], 'w') as f:
        f.write(render_html(record, run_dir))
    return paths


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Regenerate the InspectLoop report for a run.')
    ap.add_argument('run_dir')
    args = ap.parse_args(argv)
    with open(os.path.join(args.run_dir, 'mission_record.json')) as f:
        record = json.load(f)
    for k, p in write_reports(record, args.run_dir).items():
        print(f'{k}: {p}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
