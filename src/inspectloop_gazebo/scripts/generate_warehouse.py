#!/usr/bin/env python3
"""Generate the InspectLoop warehouse world, panel models and Nav2 map from one layout file.

Usage:
    generate_warehouse.py --layout config/warehouse_layout.yaml \
        --world-out worlds/warehouse.sdf --models-out models \
        --map-out ../inspectloop_navigation/maps/warehouse

Generating the occupancy map from the same geometry that builds the Gazebo world
guarantees the map is consistent with the simulated environment (no SLAM drift),
which keeps the demo reproducible. The map only contains obstacles that intersect
the LiDAR scan plane (all objects here are floor-standing).
"""
import argparse
import math
import os
import sys
import zlib
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
import yaml

# Gauge panel texture layout (pixels). Mirrored by inspectloop_vision/gauge_reader.py
# (enforced by the vision unit tests). Physical panel: 0.40 m x 0.544 m -> 0.0008 m/px.
TEX_W, TEX_H = 500, 680
MARKER_BOX = (160, 40, 340, 220)      # x0, y0, x1, y1 of the ArUco tag (0.144 m, 30 px cells)
QUIET = 30                            # white quiet zone around the tag (one marker cell)
DIAL_CENTER = (250, 465)
DIAL_R = 190                          # px (0.152 m)
ANGLE_MIN_DEG, ANGLE_MAX_DEG = -135.0, 135.0   # clockwise from 12 o'clock
VALUE_MIN, VALUE_MAX = 0.0, 10.0      # bar
LIDAR_SCAN_Z = 0.245                  # base_link z 0.08 + lidar_link z 0.165 (URDF)

@dataclass
class Box:
    name: str
    x: float
    y: float
    z: float
    sx: float
    sy: float
    sz: float
    yaw: float = 0.0
    color: Tuple[float, float, float] = (0.6, 0.6, 0.6)
    in_map: bool = True


def make_aruco(marker_id: int, size_px: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, size_px)


def value_to_angle_deg(v: float) -> float:
    return ANGLE_MIN_DEG + (v - VALUE_MIN) / (VALUE_MAX - VALUE_MIN) * (ANGLE_MAX_DEG - ANGLE_MIN_DEG)


def _polar(cx, cy, r, ang_deg):
    """Point at radius r and angle measured clockwise from 12 o'clock (image y down)."""
    a = math.radians(ang_deg)
    return int(round(cx + r * math.sin(a))), int(round(cy - r * math.cos(a)))


def make_gauge_texture(tag_id: int, label: str, value: float) -> np.ndarray:
    """Render an analog 0-10 bar pressure gauge with an ArUco id tag above it."""
    ss = 4  # supersample for smooth anti-aliased lines
    W, H = TEX_W * ss, TEX_H * ss
    img = np.full((H, W, 3), 235, np.uint8)
    cv2.rectangle(img, (6 * ss, 6 * ss), (W - 6 * ss, H - 6 * ss), (60, 60, 60), 4 * ss)
    cx, cy, R = DIAL_CENTER[0] * ss, DIAL_CENTER[1] * ss, DIAL_R * ss
    # bezel + white face
    cv2.circle(img, (cx, cy), R, (40, 40, 40), -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), int(R * 0.955), (250, 250, 250), -1, cv2.LINE_AA)
    # red alarm zone 8..10 bar
    cv2.ellipse(img, (cx, cy), (int(R * 0.90), int(R * 0.90)), -90, value_to_angle_deg(8.0),
                value_to_angle_deg(10.0), (40, 40, 210), int(0.05 * R), cv2.LINE_AA)
    # ticks
    for k in range(0, 51):
        v = VALUE_MIN + k * 0.2
        ang = value_to_angle_deg(v)
        major = k % 5 == 0
        r0 = R * (0.74 if major else 0.80)
        cv2.line(img, _polar(cx, cy, r0, ang), _polar(cx, cy, R * 0.88, ang), (20, 20, 20),
                 int((0.022 if major else 0.009) * R), cv2.LINE_AA)
        if major:
            txt = str(int(round(v)))
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.85 * ss, 2 * ss)
            px, py = _polar(cx, cy, R * 0.63, ang)
            cv2.putText(img, txt, (px - tw // 2, py + th // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.85 * ss,
                        (20, 20, 20), 2 * ss, cv2.LINE_AA)
    (tw, _), _ = cv2.getTextSize('bar', cv2.FONT_HERSHEY_SIMPLEX, 0.8 * ss, 2 * ss)
    cv2.putText(img, 'bar', (cx - tw // 2, int(cy + 0.45 * R)), cv2.FONT_HERSHEY_SIMPLEX,
                0.8 * ss, (60, 60, 60), 2 * ss, cv2.LINE_AA)
    # needle: tapered dark polygon + short tail + hub
    ang = value_to_angle_deg(value)
    tip = _polar(cx, cy, R * 0.82, ang)
    tail = _polar(cx, cy, R * 0.10, ang + 180.0)
    w = 0.035 * R
    a = math.radians(ang)
    nx, ny = math.cos(a), math.sin(a)   # perpendicular to the needle direction
    base1 = (int(cx + w * nx), int(cy + w * ny))
    base2 = (int(cx - w * nx), int(cy - w * ny))
    cv2.fillConvexPoly(img, np.array([tip, base1, tail, base2], np.int32), (15, 15, 15),
                       cv2.LINE_AA)
    cv2.circle(img, (cx, cy), int(0.07 * R), (15, 15, 15), -1, cv2.LINE_AA)
    img = cv2.resize(img, (TEX_W, TEX_H), interpolation=cv2.INTER_AREA)
    # id tag (drawn at final resolution so the marker cells stay crisp)
    x0, y0, x1, y1 = MARKER_BOX
    img[y0 - QUIET:y1 + QUIET, x0 - QUIET:x1 + QUIET] = 255
    img[y0:y1, x0:x1] = cv2.cvtColor(make_aruco(tag_id, x1 - x0), cv2.COLOR_GRAY2BGR)
    cv2.putText(img, label, (18, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (20, 20, 20), 2,
                cv2.LINE_AA)
    return img


def gauge_model_sdf(tag_id: int, size: Sequence[float]) -> str:
    d, w, h = size
    tex = f'model://gauge_panel_{tag_id}/materials/textures/gauge_{tag_id}.png'
    return f"""<?xml version="1.0"?>
<!-- Generated by generate_warehouse.py: analog pressure gauge panel, tag {tag_id}. -->
<!-- The printed face is the +X face of the box. -->
<sdf version="1.9">
  <model name="gauge_panel_{tag_id}">
    <static>true</static>
    <link name="panel">
      <collision name="collision">
        <geometry><box><size>{d} {w} {h}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{d} {w} {h}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr><metal>
            <albedo_map>{tex}</albedo_map>
            <metalness>0.0</metalness>
            <roughness>0.9</roughness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def model_config(name: str, desc: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>InspectLoop</name></author>
  <description>{desc}</description>
</model>
"""


def box_sdf(b: Box, static=True) -> str:
    r, g, bl = b.color
    return f"""    <model name="{b.name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{b.x:.4f} {b.y:.4f} {b.z:.4f} 0 0 {b.yaw:.6f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{b.sx} {b.sy} {b.sz}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{b.sx} {b.sy} {b.sz}</size></box></geometry>
          <material><ambient>{r} {g} {bl} 1</ambient><diffuse>{r} {g} {bl} 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>
"""


def rack_sdf(name: str, x: float, y: float, size: Sequence[float]) -> str:
    """A pallet rack: solid kick-plate (LiDAR-visible), uprights, shelves and some cargo."""
    sx, sy, sz = size
    kick_h = 0.35
    parts: List[str] = []

    def part(pname, px, py, pz, bx, by, bz, rgb, collide=True):
        r, g, b = rgb
        geo = f'<geometry><box><size>{bx} {by} {bz}</size></box></geometry>'
        col = f'<collision name="{pname}_c"><pose>{px} {py} {pz} 0 0 0</pose>{geo}</collision>' if collide else ''
        parts.append(
            f'        {col}<visual name="{pname}_v"><pose>{px} {py} {pz} 0 0 0</pose>{geo}'
            f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>')

    part('kick', 0, 0, kick_h / 2, sx, sy, kick_h, (0.20, 0.32, 0.55))
    for i, ux in enumerate((-sx / 2 + 0.05, 0.0, sx / 2 - 0.05)):
        for j, uy in enumerate((-sy / 2 + 0.05, sy / 2 - 0.05)):
            part(f'upright_{i}_{j}', ux, uy, sz / 2, 0.08, 0.08, sz, (0.15, 0.25, 0.50))
    for k, sh in enumerate((0.9, 1.45, sz - 0.03)):
        part(f'shelf_{k}', 0, 0, sh, sx, sy, 0.06, (0.90, 0.50, 0.10))
    rng = np.random.default_rng(zlib.crc32(name.encode()))
    for k, sh in enumerate((0.35, 0.93, 1.48)):
        for c in range(3):
            if rng.random() < 0.25:
                continue
            cx = -sx / 2 + 0.7 + c * 1.3
            part(f'cargo_{k}_{c}', cx, 0, sh + 0.22, 0.9, 0.75, 0.40, (0.62, 0.48, 0.30), collide=False)
    body = '\n'.join(parts)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
{body}
      </link>
    </model>
"""


def collect_obstacles(layout: Dict) -> Tuple[List[Box], List[Dict]]:
    boxes: List[Box] = []
    w = layout['walls']
    t, h = w['thickness'], w['height']
    cx, cy = (w['x_min'] + w['x_max']) / 2, (w['y_min'] + w['y_max']) / 2
    lx, ly = w['x_max'] - w['x_min'], w['y_max'] - w['y_min']
    wall_rgb = (0.75, 0.75, 0.72)
    boxes += [
        Box('wall_north', cx, w['y_max'] - t / 2, h / 2, lx, t, h, color=wall_rgb),
        Box('wall_south', cx, w['y_min'] + t / 2, h / 2, lx, t, h, color=wall_rgb),
        Box('wall_east', w['x_max'] - t / 2, cy, h / 2, t, ly, h, color=wall_rgb),
        Box('wall_west', w['x_min'] + t / 2, cy, h / 2, t, ly, h, color=wall_rgb),
    ]
    rs = layout['racks']['size']
    for r in layout['racks']['items']:
        boxes.append(Box(r['name'], r['xy'][0], r['xy'][1], rs[2] / 2, rs[0], rs[1], rs[2]))
    ps = layout['pillars']['size']
    for p in layout['pillars']['items']:
        boxes.append(Box(p['name'], p['xy'][0], p['xy'][1], ps[2] / 2, *ps, color=(0.85, 0.75, 0.1)))
    pl = layout['pallets']['size']
    for p in layout['pallets']['items']:
        boxes.append(Box(p['name'], p['xy'][0], p['xy'][1], pl[2] / 2, *pl, color=(0.55, 0.40, 0.22)))
    g = layout['gauges']
    gd, gw, gh = g['size']
    for p in g['items']:
        boxes.append(Box(f"gauge_panel_{p['tag_id']}", p['xy'][0], p['xy'][1], g['center_z'],
                         gd, gw, gh, yaw=p['normal_yaw']))
    for o in layout.get('occluders', []):
        sx, sy, sz = o['size']
        zb = float(o.get('z_bottom', 0.0))
        boxes.append(Box(o['name'], o['xy'][0], o['xy'][1], zb + sz / 2, sx, sy, sz,
                         color=tuple(o.get('color', (0.4, 0.4, 0.4))),
                         in_map=zb < LIDAR_SCAN_Z))
        if 'bracket_to_y' in o:   # horizontal support arm above the scan plane
            y0, y1 = o['xy'][1], o['bracket_to_y']
            boxes.append(Box(o['name'] + '_bracket', o['xy'][0], (y0 + y1) / 2, zb + sz - 0.02,
                             0.03, abs(y1 - y0), 0.03, color=(0.2, 0.2, 0.2), in_map=False))
    return boxes, layout['gauges']['items']


def rasterize(boxes: List[Box], res: float, origin: Sequence[float], size: Sequence[float]) -> np.ndarray:
    """Return an occupancy grid (row 0 = max y, as in a PGM image). 1 = occupied."""
    nx, ny = int(round(size[0] / res)), int(round(size[1] / res))
    xs = origin[0] + (np.arange(nx) + 0.5) * res
    ys = origin[1] + (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)          # gy increases with row index
    occ = np.zeros((ny, nx), bool)
    for b in boxes:
        if not b.in_map:
            continue
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        dx, dy = gx - b.x, gy - b.y
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        # inflate by half a cell so thin objects (panels) always occupy >= 1 cell
        hx, hy = b.sx / 2 + res / 2, b.sy / 2 + res / 2
        occ |= (np.abs(lx) <= hx) & (np.abs(ly) <= hy)
    return occ[::-1, :]


def world_sdf(layout: Dict, boxes: List[Box]) -> str:
    racks = {r['name'] for r in layout['racks']['items']}
    rs = layout['racks']['size']
    models: List[str] = []
    for b in boxes:
        if b.name in racks:
            models.append(rack_sdf(b.name, b.x, b.y, rs))
        elif b.name.startswith('gauge_panel_'):
            models.append(f"""    <include>
      <uri>model://{b.name}</uri>
      <name>{b.name}</name>
      <pose>{b.x:.4f} {b.y:.4f} {b.z:.4f} 0 0 {b.yaw:.6f}</pose>
    </include>
""")
        else:
            models.append(box_sdf(b))
    # Ceiling lights above each panel give consistent, repeatable illumination.
    lights = []
    for p in layout['gauges']['items']:
        x, y = p['xy']
        ox, oy = math.cos(p['normal_yaw']) * 1.2, math.sin(p['normal_yaw']) * 1.2
        lights.append(f"""    <light type="point" name="gauge_light_{p['tag_id']}">
      <pose>{x + ox:.3f} {y + oy:.3f} 2.6 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>8</range><constant>0.4</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
""")
    return f"""<?xml version="1.0"?>
<!-- GENERATED by inspectloop_gazebo/scripts/generate_warehouse.py - do not edit by hand. -->
<sdf version="1.9">
  <world name="{layout['world_name']}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.70 0.75 0.80 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
{''.join(lights)}
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry>
          <material><ambient>0.45 0.46 0.48 1</ambient><diffuse>0.45 0.46 0.48 1</diffuse></material>
        </visual>
      </link>
    </model>

{''.join(models)}
  </world>
</sdf>
"""


def write_map(occ: np.ndarray, res: float, origin: Sequence[float], map_out: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(map_out)), exist_ok=True)
    img = np.where(occ, 0, 254).astype(np.uint8)
    pgm = map_out + '.pgm'
    h, w = img.shape
    with open(pgm, 'wb') as f:
        f.write(f'P5\n# InspectLoop warehouse (generated)\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    with open(map_out + '.yaml', 'w') as f:
        f.write(f"""# GENERATED by generate_warehouse.py from warehouse_layout.yaml
image: {os.path.basename(pgm)}
mode: trinary
resolution: {res}
origin: [{origin[0]}, {origin[1]}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--layout', required=True)
    ap.add_argument('--world-out')
    ap.add_argument('--models-out')
    ap.add_argument('--map-out', help='path prefix (without extension) for .pgm/.yaml')
    args = ap.parse_args(argv)

    with open(args.layout) as f:
        layout = yaml.safe_load(f)
    boxes, panels = collect_obstacles(layout)

    if args.models_out:
        for p in panels:
            tag = p['tag_id']
            mdir = os.path.join(args.models_out, f'gauge_panel_{tag}')
            os.makedirs(os.path.join(mdir, 'materials', 'textures'), exist_ok=True)
            cv2.imwrite(os.path.join(mdir, 'materials', 'textures', f'gauge_{tag}.png'),
                        make_gauge_texture(tag, p['id'], p['value']))
            with open(os.path.join(mdir, 'model.sdf'), 'w') as f:
                f.write(gauge_model_sdf(tag, layout['gauges']['size']))
            with open(os.path.join(mdir, 'model.config'), 'w') as f:
                f.write(model_config(f'gauge_panel_{tag}',
                                     f'Analog pressure gauge {p["id"]} (0-10 bar), tag {tag}'))
    if args.world_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.world_out)), exist_ok=True)
        with open(args.world_out, 'w') as f:
            f.write(world_sdf(layout, boxes))
    if args.map_out:
        m = layout['map']
        occ = rasterize(boxes, m['resolution'], m['origin'], m['size'])
        write_map(occ, m['resolution'], m['origin'], args.map_out)
    print(f'generated {len(boxes)} objects, {len(panels)} gauges')
    return 0


if __name__ == '__main__':
    sys.exit(main())
