import os

import pytest

from inspectloop_mission.mission_config import (alarm_state, load_mission, MissionConfigError,
                                                parse_mission)

CFG = os.path.join(os.path.dirname(__file__), '..', 'config')


def test_default_mission_loads():
    m = load_mission(os.path.join(CFG, 'gauges.yaml'))
    assert [g.id for g in m.gauges] == ['PG-11', 'PG-12', 'PG-13', 'PG-14']
    assert [g.tag_id for g in m.gauges] == [11, 12, 13, 14]
    assert m.home is not None and m.home.x == -10.0


def test_alarm_state():
    g = load_mission(os.path.join(CFG, 'gauges.yaml')).gauges[0]
    assert alarm_state(g, 4.0) == 'NORMAL'
    assert alarm_state(g, 8.5) == 'HIGH'
    assert alarm_state(g, 0.5) == 'LOW'


def _g(**kw):
    base = {'id': 'G', 'tag_id': 1, 'mount': {'x': 0, 'y': 0, 'normal_yaw': 0}}
    base.update(kw)
    return base


@pytest.mark.parametrize('data', [
    {}, {'mission': {}, 'gauges': []},
    {'mission': {}, 'gauges': [{'id': 'a', 'tag_id': 1}]},
    {'mission': {}, 'gauges': [_g(tag_id='x')]},
    {'mission': {}, 'gauges': [_g(mount={'x': 'bad', 'y': 0, 'normal_yaw': 0})]},
    {'mission': {}, 'gauges': [_g(), _g()]},
    {'mission': {}, 'gauges': [_g(alarm_low=5, alarm_high=2)]},
    {'mission': {}, 'gauges': [_g(mount={'x': float('nan'), 'y': 0, 'normal_yaw': 0})]},
])
def test_invalid_configs_rejected(data):
    with pytest.raises(MissionConfigError):
        parse_mission(data)


def test_missing_file():
    with pytest.raises(MissionConfigError):
        load_mission('/nonexistent/mission.yaml')
