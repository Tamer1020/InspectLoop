# InspectLoop portfolio films V2

[Main film](https://github.com/Tamer1020/InspectLoop/releases/download/v3.2.0/InspectLoop-Demo-V2.mp4) · [PG-12 portrait cut](https://github.com/Tamer1020/InspectLoop/releases/download/v3.2.0/InspectLoop-PG12-V2.mp4) · [Cover image](../../../docs/media/v2/InspectLoop-V2-Cover.jpg)

## Recordings and clocks

Gazebo and RViz ran together in [capture job 43722286660](https://github.com/Tamer1020/InspectLoop/actions/runs/43722286660). Both X11 recordings are 1280 × 720, 8 fps and 467.5 s long. The edit uses the two original MKVs. Every displayed movement plays at recording speed. The camera panels are recorded still observations from [mission 20261115_140724_e358](../../../docs/media/capture/20261115_140724_e358/mission_record.json). The safety experiment was a separate run.

| Source | SHA-256 |
|---|---|
| Original job ZIP | `122577292d217efe1613ff54bab6c8a6c9367a3c1193e40655ab7c65515d28ee` |
| gazebo.mkv | `303210a6ce4f8ed145963b5a857ffb3b94b7f5b205502a4a84b742c19ca9b56d` |
| rviz.mkv | `121f0a085ca929799d3e4ebae9fad05cf49cc70575c88529f9469e39c046108c` |
| launch.log | `0d37cc18899852e03eebd352ff5729b560b8778d2855ecb3dc2b7a0501a2c87f` |

**REC** gives elapsed time in the paired source recordings. The two recorders started in the same launch step and each contains 3,740 evenly spaced frames. The MKVs carry relative timestamps. **UTC** labels come from the [timestamped mission-manager log extract](mission-events.csv). Camera bursts use ROS simulation time from the [PG-12 observation record](../../../docs/media/capture/20261115_140724_e358/gauge_PG-12.json). The chart uses elapsed time from its own odometry recorder.

The actual RViz feedback text provides independent synchronization landmarks. UI updates have display latency. The numerical precision of the UTC log describes ROS events, not display refresh accuracy.

| Goal | Nav2 accepted (UTC) | RViz active (REC) | Nav2 succeeded (UTC) | RViz reached (REC) |
|---|---|---:|---|---:|
| PG-12 V0, frontal | 14:07:58.214 | 47.625 s | 14:08:52.586 | 102.000 s |
| PG-12 V3, −45° at 1.50 m | 14:09:05.374 | 114.875 s | 14:09:24.957 | 134.375 s |
| PG-12 V8, −45° at 1.20 m | 14:09:36.668 | 146.125 s | 14:09:39.026 | 148.500 s |

At V3, the camera returned 6.8056 bar with four valid frames and a blocking `TOO_FAR` flag. At V8, five valid frames produced an accepted median of 6.7833 bar. The evaluator compared that result with the 6.8000 bar simulation reference after the mission. Distances on screen are nominal target standoffs. The [mission record](../../../docs/media/capture/20261115_140724_e358/gauge_PG-12.json) also reports the measured arrival poses.

## Main film

| Edit time | Picture | Source evidence |
|---|---|---|
| 00:00–00:06 | Warehouse departure | Gazebo and RViz [32.000, 38.000) s. |
| 00:06–00:12 | PG-11 approach | Gazebo and RViz [38.000, 44.000) s. |
| 00:12–00:17 | PG-11 reading | [PG-11 observation 1](../../../docs/media/capture/20261115_140724_e358/images/PG-11_V0_t00022.20_annotated.png), burst 22.2–22.6 s simulation time. Policy event 14:07:58.068 UTC. |
| 00:17–00:23.5 | PG-12 frontal arrival | Gazebo and RViz [96.500, 103.000) s. |
| 00:23.5–00:27.5 | Frontal burst 1 | [PG-12 observation 1](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V0_t00052.50_annotated.png), burst 52.5–54.0 s simulation time. Policy event 14:08:58.192 UTC. |
| 00:27.5–00:31.5 | Frontal burst 2 | [PG-12 observation 2](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V0_t00055.70_annotated.png), burst 55.7–57.7 s simulation time. Policy event 14:09:05.306 UTC. |
| 00:31.5–00:53 | First side-view goal | Gazebo and RViz [113.875, 135.375) s. |
| 00:53–00:57.5 | Distance rejection | [PG-12 observation 3](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V3_t00070.50_annotated.png), burst 70.5–73.8 s simulation time. Policy event 14:09:36.541 UTC. |
| 00:57.5–01:02.25 | Closer side-view goal | Gazebo and RViz [144.750, 149.500) s. |
| 01:02.25–01:08.25 | Accepted side view | [PG-12 observation 4](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V8_t00077.70_annotated.png), burst 77.2–78.5 s simulation time. Policy event 14:09:44.768 UTC. |
| 01:08.25–01:11.75 | PG-13 final approach | Gazebo and RViz [277.500, 281.000) s. |
| 01:11.75–01:15.5 | PG-13 unresolved | Gazebo [294.875, 298.625) s. Paired with the [last camera observation](../../../docs/media/capture/20261115_140724_e358/images/PG-13_V2_t00146.40_annotated.png), 146.4–148.5 s simulation time. Decision 14:12:07.835 UTC. |
| 01:15.5–01:20.25 | PG-14 high alarm | [PG-14 observation 1](../../../docs/media/capture/20261115_140724_e358/images/PG-14_V0_t00170.20_annotated.png), burst 170.2–172.5 s simulation time. Policy event 14:12:56.981 UTC. |
| 01:20.25–01:27.75 | Separate LiDAR fault run | [Odometry](../../../docs/evidence/astra_v3_1_live/f3_tiny_inc/odom_speed.txt) and [test report](../../../docs/evidence/astra_v3_2/colcon_test_result.txt). |
| 01:27.75–01:33.5 | Mission summary over return leg | Gazebo [447.000, 452.750) s. [Mission report](../../../docs/media/capture/20261115_140724_e358/mission_report.md) and [offline evaluation](../../../docs/media/capture/20261115_140724_e358/evaluation.json). |
| 01:33.5–01:36.5 | Source code | Repository URL and filmed mission identifier. |

## Portrait cut

| Edit time | Picture | Source evidence |
|---|---|---|
| 00:00–00:03 | Frontal burst 1 | [PG-12 observation 1](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V0_t00052.50_annotated.png), burst 52.5–54.0 s simulation time. Policy event 14:08:58.192 UTC. |
| 00:03–00:05.75 | Frontal burst 2 | [PG-12 observation 2](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V0_t00055.70_annotated.png), burst 55.7–57.7 s simulation time. Policy event 14:09:05.306 UTC. |
| 00:05.75–00:21.25 | First side-view goal | Gazebo and RViz [117.750, 133.250) s. |
| 00:21.25–00:24.75 | Distance rejection | [PG-12 observation 3](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V3_t00070.50_annotated.png), burst 70.5–73.8 s simulation time. Policy event 14:09:36.541 UTC. |
| 00:24.75–00:28.75 | Closer side-view goal | Gazebo and RViz [145.125, 149.125) s. |
| 00:28.75–00:34.25 | Accepted side view | [PG-12 observation 4](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V8_t00077.70_annotated.png), burst 77.2–78.5 s simulation time. Policy event 14:09:44.768 UTC. |

## Export and reproduction

The main film is 1920 × 1080 and 96.50 s. The portrait cut is 1080 × 1920 and 34.25 s. Both are silent, use the original 8 fps cadence, and have front-loaded MP4 metadata for playback. The H.264 encoder uses high-quality variable bitrate with an 8 Mbps ceiling and short groups of pictures for seeking through the reposition. The cover pairs a Gazebo frame at REC 122.000 s with the final rectified PG-12 dial.

Use Python 3.10 or later, Pillow, FFmpeg with libx264, Open Sans and DejaVu Sans Mono. Obtain `gazebo.mkv` and `rviz.mkv` from the capture job or the preserved review source archive. Run this from the repository root with the directory containing both files.

```bash
python3 tools/portfolio/v2/render.py recordings
```

The renderer verifies the hashes of both recordings before editing. It reads the versioned camera images and mission evidence and writes the two MP4 files and cover to `docs/media/v2/`. [Editor source](render.py) · [Original capture workflow](https://github.com/Tamer1020/InspectLoop/blob/037aba5896a38944da6e0d4404a2680b6e1e7c05/.github/workflows/portfolio-capture.yml) · [Original film and capture notes](../README.md)

## README figures

- **Camera comparison.** The two panels are crops of the recorded [V0 RGB frame](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V0_t00055.70_raw.png) and [V8 RGB frame](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V8_t00077.70_raw.png). Captions give the corresponding burst-level valid frame counts. The [intermediate V3 observation](../../../docs/media/capture/20261115_140724_e358/images/PG-12_V3_t00070.50_annotated.png) documents the second move.
- **Navigation pair.** Gazebo and RViz are cropped from frame 976 (zero-based, REC 122.000 s) of their respective source recordings during the V0-to-V3 goal.
- **System diagram.** The figure follows the inspection and velocity-command paths in the [ROS 2 architecture](../../../docs/ARCHITECTURE.md). The gate receives both the navigation command and LiDAR data.
- **Pressure chart.** The markers use the accepted pressures in the filmed round’s [offline evaluation](../../../docs/media/capture/20261115_140724_e358/evaluation.json). The PG-13 row has no pressure marker. The PG-14 tick is its 8.0 bar high alarm limit.
