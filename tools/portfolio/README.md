# Reproduce the InspectLoop films

The main film runs 93 seconds at 1920 × 1080. The 30-second vertical cut runs at 1080 × 1920. Both are silent, captioned H.264 MP4 files encoded at 24 fps. GUI recordings were acquired at 8 fps.

## Source material

The navigation clips, robot camera images, rectified dials, policy decisions and final readings come from [the same completed mission](../../docs/media/capture/20261115_140724_e358/mission_record.json). The result figure uses the [offline evaluator output](../../docs/media/capture/20261115_140724_e358/evaluation.json). The report insert renders the mission's HTML report. The safety segment uses a [separate LiDAR fault run](../../docs/evidence/astra_v3_1_live/f3_tiny_inc/odom_speed.txt). Its measured speed windows are relative to that recorder becoming ready.

| Main film time | Sequence |
|---|---|
| 00:00–00:14 | Warehouse introduction and synchronized Gazebo / RViz navigation |
| 00:14–00:21 | PG-11 and the rectified analog dial |
| 00:21–00:55 | PG-12 frontal rejection, retry, alternate angle, distance rejection and accepted closer view |
| 00:55–01:09 | PG-13 unresolved and PG-14 high-pressure alarm |
| 01:09–01:28 | Separate LiDAR fault experiment, mission report and results |
| 01:28–01:33 | Project and repository credits |

## Render

On Debian or Ubuntu, install FFmpeg, Python 3, Pillow, Open Sans and DejaVu fonts. From the repository root:

```bash
sudo apt install ffmpeg python3-pil fonts-open-sans fonts-dejavu-core
python3 tools/portfolio/render.py --output dist/portfolio
```

The command writes both MP4 files and refreshes the publication images in `docs/media/`. To regenerate the mission report insert, install WeasyPrint and Poppler utilities and add `--refresh-report`. Scene order and durations are in the `MAIN` and `SHORT` lists in [render.py](render.py). A regular render uses the committed clips and requires no ROS installation.

## Repeat the GUI capture

The [hosted recording workflow](https://github.com/Tamer1020/InspectLoop/blob/037aba5896a38944da6e0d4404a2680b6e1e7c05/.github/workflows/portfolio-capture.yml) specifies the ROS build, virtual displays, follow camera, launch options and recorder stop condition. The [capture notes](../../docs/media/capture/README.md) identify the exact intervals selected against the navigation log. To reproduce the crops from the two original MKVs in the extracted recording artifact:

```bash
python3 tools/portfolio/select_capture.py --first /path/to/capture --move /path/to/capture --move-start 120
```

The trimmed clips and original mission evidence remain in the repository after the temporary recording artifact expires. To run an inspection locally, follow [the demo guide](../../docs/RUN_DEMO.md).
