"""V3 (Astra V2 review, F9): report image links must survive moving/copying a run directory."""
import os
import re
import shutil

from inspectloop_mission import report


def _make_run(root):
    run = os.path.join(root, 'orig', '20261108_101010_ab12')
    os.makedirs(os.path.join(run, 'images'))
    img = os.path.join(run, 'images', 'PG-11_V0_t0012.00_annotated.png')
    with open(img, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
    obs = {'index': 1, 'value': 4.2, 'sigma': 0.05, 'quality_flags': [], 'decision': 'ACCEPT',
           'reason': 'ok', 'frames_valid': 5, 'frames_received': 5,
           'annotated_image_path': img}
    rec = {'mission_id': 'm', 'mission_name': 'portability', 'state': 'COMPLETED',
           'run_dir': run, 'parameters': {},
           'gauges': [{'id': 'PG-11', 'unit': 'bar',
                       'viewpoints': [{'viewpoint': {'index': 0, 'label': 'V0', 'x': 1.0,
                                                     'y': 2.0, 'yaw': 0.0, 'distance': 1.5,
                                                     'angle_deg': 0.0},
                                       'navigation': {'outcome': 'SUCCEEDED', 'attempts': [],
                                                      'total_time_s': 1.0},
                                       'observations': [obs]}],
                       'timeline': [{'decision': 'ACCEPT'}],
                       'result': {'status': 'ACCEPTED', 'value': 4.2, 'sigma_heuristic': 0.05,
                                  'alarm': 'NORMAL', 'observations': 1, 'viewpoints_used': 1,
                                  'annotated_image_path': img}}]}
    return run, rec


def _links(text):
    return re.findall(r"<img src='([^']+)'", text) + re.findall(r'\]\(([^)]+\.png)\)', text)


def _assert_links_resolve(run_dir, paths):
    for kind in ('markdown', 'html'):
        links = _links(open(paths[kind]).read())
        assert links, f'no image link in the {kind} report'
        for ln in links:
            assert not os.path.isabs(ln), ln
            assert os.path.isfile(os.path.join(run_dir, ln)), f'{kind}: broken link {ln}'


def test_links_resolve_in_the_original_location(tmp_path):
    run, rec = _make_run(str(tmp_path))
    _assert_links_resolve(run, report.write_reports(rec, run))


def test_links_resolve_after_copying_the_run_directory(tmp_path):
    run, rec = _make_run(str(tmp_path))
    moved = os.path.join(str(tmp_path), 'elsewhere', 'deep', 'copy_of_run')
    shutil.copytree(run, moved)
    shutil.rmtree(run)                          # the original location no longer exists
    _assert_links_resolve(moved, report.write_reports(rec, moved))


def test_already_relative_paths_are_kept_relative_to_the_run_dir(tmp_path, monkeypatch):
    run, rec = _make_run(str(tmp_path))
    for g in rec['gauges']:
        for v in g['viewpoints']:
            for o in v['observations']:
                o['annotated_image_path'] = os.path.relpath(o['annotated_image_path'], run)
    monkeypatch.chdir(str(tmp_path))            # cwd != run dir: relpath(cwd) would be wrong
    _assert_links_resolve(run, report.write_reports(rec, run))


def test_rel_unit_cases(tmp_path):
    run = os.path.join(str(tmp_path), 'new')
    assert report._rel('images/a.png', run) == 'images/a.png'
    assert report._rel('/old/place/run/images/a.png', run, '/old/place/run') == 'images/a.png'
    assert report._rel(os.path.join(run, 'images', 'a.png'), run) == os.path.join('images', 'a.png')
    assert report._rel('', run) == ''
