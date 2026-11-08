# Setup

Verified environment (2026-11-08): Windows 11 + WSL2, Ubuntu 24.04.5 LTS, ROS 2 Jazzy,
Gazebo Harmonic (gz-sim 8.x via the ROS `gz_*_vendor` packages), Nav2 1.3.13, ros_gz 1.0.24,
OpenCV 4.6.0 (python3-opencv), GCC 13.3, CMake 3.28, Python 3.12, 16 cores / 15 GiB RAM.

## 1. System packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-cv-bridge python3-opencv python3-numpy python3-yaml python3-pytest \
  python3-colcon-common-extensions
```

`ros-jazzy-ros-gz` pulls Gazebo Harmonic in through the vendor packages; a separate Gazebo install
is not needed. `gz_ros2_control` is **not** required (the robot uses Gazebo's DiffDrive system).

Optional: `sudo apt install python3-rosdep && sudo rosdep init && rosdep update`, then
`rosdep install --from-paths src --ignore-src -y` resolves everything from the `package.xml` files
(this is what CI does).

## 2. Build

```bash
cd ~/InspectLoop            # the workspace root (contains src/)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Test

```bash
colcon test
colcon test-result --verbose      # expected: 0 errors, 0 failures
```

## 4. Regenerating the world (only if you edit the layout)

```bash
cd src/inspectloop_gazebo
python3 scripts/generate_warehouse.py --layout config/warehouse_layout.yaml \
  --world-out worlds/warehouse.sdf --models-out models \
  --map-out ../inspectloop_navigation/maps/warehouse
```

`test_world_generation.py` fails if the committed world, textures or map are out of date.

## 5. WSL2 notes

* Headless runs use `gz sim -s --headless-rendering` (EGL offscreen); camera and GPU LiDAR render
  correctly under WSL2 without a display.
* GUI runs use WSLg. If Qt windows misbehave, `export QT_QPA_PLATFORM=xcb`.
* Windows path to the workspace: `\\wsl.localhost\Ubuntu-24.04\home\tamer\InspectLoop`
  (use `\\wsl$\Ubuntu-24.04\...` on older Windows builds; the distro name is shown by `wsl -l -v`).

## 6. Publishing to GitHub (Tamer1020/InspectLoop)

The repository is a local git repo with a clean history on `main`. It was not pushed from the
build environment because no GitHub credentials were available there. To publish:

```bash
# option A: GitHub CLI
sudo apt install gh && gh auth login
cd ~/InspectLoop
gh repo create Tamer1020/InspectLoop --public --source=. --remote=origin --push

# option B: create an empty public repo "InspectLoop" on github.com (no README/license), then
cd ~/InspectLoop
git remote add origin https://github.com/Tamer1020/InspectLoop.git
git push -u origin main          # authenticate with a personal access token
```

The CI workflow (`.github/workflows/ci.yml`) then builds and runs all tests in a `ros:jazzy-ros-base`
container. (It has not been executed yet; the simulation itself is not run in CI.)
If you received the portfolio ZIP instead of the repo, restore full history with
`git clone InspectLoop/InspectLoop.git.bundle InspectLoop-repo`.
