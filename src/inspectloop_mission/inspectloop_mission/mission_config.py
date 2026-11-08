"""Gauge inspection mission definition: loading and validation (ROS independent)."""
from dataclasses import dataclass
import math
from typing import List, Optional

import yaml


class MissionConfigError(ValueError):
    pass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def as_dict(self):
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw}


@dataclass(frozen=True)
class GaugeSpec:
    """What the operator knows about a gauge: where it is mounted (asset registry / drawings),
    its tag id and its scale. Its current reading is what the robot must find out."""
    id: str
    tag_id: int
    x: float
    y: float
    normal_yaw: float
    description: str = ''
    unit: str = 'bar'
    alarm_low: Optional[float] = None
    alarm_high: Optional[float] = None


@dataclass
class MissionConfig:
    name: str
    gauges: List[GaugeSpec]
    home: Optional[Pose2D] = None
    notes: str = ''


def _num(d, key, where):
    try:
        v = float(d[key])
    except (KeyError, TypeError, ValueError) as e:
        raise MissionConfigError(f'{where}: {key} must be a number') from e
    if not math.isfinite(v):
        raise MissionConfigError(f'{where}: {key} must be finite')
    return v


def _pose(d, where) -> Pose2D:
    yaw = _num(d, 'yaw', where) if 'yaw' in d else 0.0
    return Pose2D(_num(d, 'x', where), _num(d, 'y', where),
                  math.atan2(math.sin(yaw), math.cos(yaw)))


def parse_mission(data: dict) -> MissionConfig:
    if not isinstance(data, dict) or 'mission' not in data or 'gauges' not in data:
        raise MissionConfigError("mission file needs top-level 'mission' and 'gauges'")
    m = data['mission'] or {}
    raw = data['gauges']
    if not isinstance(raw, list) or not raw:
        raise MissionConfigError('gauges must be a non-empty list')
    seen, gauges = set(), []
    for i, g in enumerate(raw):
        where = f'gauges[{i}]'
        if not isinstance(g, dict) or 'id' not in g or 'mount' not in g or 'tag_id' not in g:
            raise MissionConfigError(f"{where}: needs 'id', 'tag_id' and 'mount'")
        gid = str(g['id'])
        if gid in seen:
            raise MissionConfigError(f'{where}: duplicate gauge id {gid}')
        seen.add(gid)
        try:
            tag = int(g['tag_id'])
        except (TypeError, ValueError) as e:
            raise MissionConfigError(f'{where}: tag_id must be an integer') from e
        mount = g['mount'] or {}
        lo = g.get('alarm_low')
        hi = g.get('alarm_high')
        if lo is not None and hi is not None and float(lo) >= float(hi):
            raise MissionConfigError(f'{where}: alarm_low must be < alarm_high')
        gauges.append(GaugeSpec(gid, tag, _num(mount, 'x', where), _num(mount, 'y', where),
                                _num(mount, 'normal_yaw', where), str(g.get('description', '')),
                                str(g.get('unit', 'bar')),
                                None if lo is None else float(lo),
                                None if hi is None else float(hi)))
    home = _pose(m['home'], 'mission.home') if m.get('home') else None
    return MissionConfig(str(m.get('name', 'gauge_inspection')), gauges, home,
                         str(m.get('notes', '')))


def load_mission(path: str) -> MissionConfig:
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except OSError as e:
        raise MissionConfigError(f'cannot read mission file {path}: {e}') from e
    except yaml.YAMLError as e:
        raise MissionConfigError(f'invalid YAML in {path}: {e}') from e
    return parse_mission(data)


def alarm_state(spec: GaugeSpec, value: float) -> str:
    if spec.alarm_high is not None and value > spec.alarm_high:
        return 'HIGH'
    if spec.alarm_low is not None and value < spec.alarm_low:
        return 'LOW'
    return 'NORMAL'
