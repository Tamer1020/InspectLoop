# Filmed mission and navigation captures

All published GUI clips, stills and gauge images come from mission [20261115_140724_e358](20261115_140724_e358/mission_report.md). Gazebo Harmonic and RViz2 ran on two 1280 × 720 X11 displays on one Ubuntu runner. The recorder captured both at 8 fps. The ROS 2 mission completed all four inspections and the return home. [Recording job](https://github.com/Tamer1020/InspectLoop/actions/runs/43722286660)

| Asset | Recording time | What it shows |
|---|---|---|
| [nav.mp4](nav.mp4) | 28–36 s | Synchronized Gazebo and RViz views during the approach to PG-11 |
| [gazebo-hero.png](gazebo-hero.png) | 31 s | Gazebo viewport crop |
| [rviz-navigation.png](rviz-navigation.png) | 29 s | Full RViz navigation display |
| [move.mp4](move.mp4) | 120–129 s | PG-12 navigation to the first alternate pose, between Nav2 acceptance at 14:09:05 UTC and success at 14:09:24 UTC |

The [log excerpt](navigation-events.log) retains the launch origin and Nav2 event timestamps. The [machine-readable mission record](20261115_140724_e358/mission_record.json) includes each observation, quality flag, viewpoint and action outcome. The [offline evaluation](20261115_140724_e358/evaluation.json) compares final readings with the simulation reference. The PNGs under the mission's images directory are the original camera and reader outputs.

[Video editing and reproduction](../../../tools/portfolio/README.md)
