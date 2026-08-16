# UR10e sphere-scan demo in Gazebo — workspace setup (Ubuntu 22.04, ROS2 Humble)

Two packages here:
- **`arm_scan_demo`** — the sphere-scan node (`move_group_interface.py`) plus the
  launch file that ties everything together.
- **`ur10e_tool_description`** — a xacro that wraps UR's own UR10e model and
  bolts a simple placeholder tool onto `tool0`, so it's visible/collides in
  Gazebo. Swap the cylinder geometry in
  `urdf/ur10e_with_tool.urdf.xacro` for your real tool's mesh once this runs.

## 1. Install ROS2 Humble (skip if already installed)
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init   # only if this is the first time rosdep has been set up on this machine
rosdep update
```

## 2. Install MoveIt2 and the UR ROS2 stack
```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ur \
  ros-humble-ur-simulation-gazebo \
  python3-pip

pip3 install numpy
```
If `ros-humble-ur-simulation-gazebo` isn't found for your exact point
release, build it from source instead:
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
```

### pymoveit2 (required)
The scan node drives MoveIt through **pymoveit2**, not `moveit_py`. There is no
apt package for it, so it has to be cloned into the workspace alongside these
two packages — without it `move_group_interface` dies on import:
```bash
cd ~/ros2_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
```

## 3. Create the workspace and drop in both packages
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# unzip/copy both arm_scan_demo/ and ur10e_tool_description/ in here, so:
#   ~/ros2_ws/src/arm_scan_demo/{package.xml,setup.py,arm_scan_demo/,launch/}
#   ~/ros2_ws/src/ur10e_tool_description/{package.xml,CMakeLists.txt,urdf/}
```

## 4. Build
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 5. Sanity-check the launch args before running
UR's launch files' argument names shift between releases, so confirm what's
actually installed before trusting `ur10e_sim_demo.launch.py`:
```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py --show-args
```
If `description_package`/`description_file` aren't the args it expects (some
versions use `safety_limits`, `description_launchfile`, etc.), edit the
`launch_arguments` dict at the top of
`arm_scan_demo/launch/ur10e_sim_demo.launch.py` to match. Also check that
`ur_moveit_config`'s own launch files build `MoveItConfigsBuilder` the same
way this file does (same idea: `ros2 pkg prefix ur_moveit_config`, then look
in its `launch/` directory) — the `.robot_description_semantic(...)` and
`.robot_description(...)` calls may need different mappings on your version.

## 6. Run it
```bash
ros2 launch arm_scan_demo ur10e_sim_demo.launch.py
```
This starts Gazebo with the UR10e + tool spawned, `ur_moveit_config`'s own
`move_group` + RViz (so you can watch it), and then — after
`scan_start_delay` seconds, once controllers are up — the scan node, which
publishes the scene markers and the waypoint `PoseArray` and then immediately
starts driving the tool around the object. There is no Enter prompt; it runs
as soon as it starts.

The delay is a launch argument, because it is racing the controller spawners
rather than waiting on them. If the log shows `Joint states are not available
yet!` or the controller spawner retrying `/controller_manager`, raise it:
```bash
ros2 launch arm_scan_demo ur10e_sim_demo.launch.py scan_start_delay:=45.0
```

### Capturing a run
Rather than selecting terminal output by hand, use the wrapper — it saves
everything about one run into `dump/<timestamp>/`:
```bash
./src/arm_scan_demo/scripts/run_scan.sh                       # ur10e_sim_demo.launch.py
./src/arm_scan_demo/scripts/run_scan.sh test.launch.py
./src/arm_scan_demo/scripts/run_scan.sh test.launch.py scan_start_delay:=45.0
```
| file | contents |
| --- | --- |
| `summary.txt` | scan tally, waypoint outcomes, collision pairs by frequency, distinct errors — printed to the terminal too |
| `console-clean.log` | full output, ANSI stripped, `\r` progress lines split into real lines |
| `console.log` | raw terminal capture |
| `environment.txt` | git commit, working-tree state, package prefixes, ROS env |
| `ros_log/` | ROS 2's own per-node logs, normally buried in `~/.ros/log/` |

It refuses to start if Gazebo is already running. A leftover `gzserver` keeps
serving `/spawn_entity`, so the new one exits 255, `spawn_entity` reports
`Entity [ur] already exists`, the controllers fail to configure, and the scan
silently executes against the previously loaded robot. Pass `--kill-stale` to
clear it automatically.

In RViz, add displays on the `base_link` frame for `/poses` (PoseArray),
`/visualization_marker` (Marker — sphere, floor, pedestals) and
`/visualization_marker_array` (MarkerArray — waypoint dots and orientation
axes) to see the scan pattern before/while it executes.

## Things you'll likely need to tune
All of these are constants at the top of `move_group_interface.py`:
- **`SPHERE_CENTER`** (currently `[0.55, 0.0, 0.45]`, in the `base_link`
  frame) and **`SPHERE_RADIUS`** — put these where your actual object is.
- **`SCAN_RADIUS`** — distance from the object centre that `scanner_frame` is
  held at. Must clear `SPHERE_RADIUS + SCANNER_COLLISION_SPHERE_RADIUS` or
  every waypoint starts in collision; the node logs the computed clearance at
  startup and errors if it goes non-positive.
- **`SCANNER_COLLISION_SPHERE_RADIUS`** — the scanner's protective envelope.
- **`NUM_POINTS`**, **`SCAN_PATTERN`** (`full_sphere` / `ring` / `bands`) and
  **`ELEVATIONS_DEG`** (used only by `bands`) if the coverage is too
  sparse/dense or clips the robot's own base.
- **`MAX_VELOCITY_SCALING`** / **`MAX_ACCELERATION_SCALING`** — both 0.15.
- **The tool geometry** in `ur10e_with_tool.urdf.xacro` — currently a plain
  grey cylinder as a placeholder. If you change its length, move
  `tool_tip_joint`'s origin to match, and keep the geometry offset so it grows
  outward from `tool0` instead of back into the wrist.

## The scanner
`ur10e_with_tool.urdf.xacro` mounts a scanner past the tool:

```
tool0 -> tool_link -> tool_tip -> scanner_mount_link -> scanner_head_link -> scanner_frame
                                                                            ^ 0.21 m from the flange
```

`scanner_frame` is the frame the scan aims: its local **+Z** points out of the
scanner's front face, and the waypoints place it `SCAN_RADIUS` from the object
centre with +Z toward it. It is `END_EFFECTOR_LINK` in the node.

Geometry is driven by xacro properties at the top of the scanner section —
`scanner_mount_offset_x` (set non-zero to hang the scanner off-axis),
`scanner_mount_offset_z`, `scanner_body_x/y/z`, and `scanner_standoff`.

**The scanner links carry visual geometry only.** All of its collision is the
sphere the node attaches to `scanner_head_link`
(`SCANNER_COLLISION_SPHERE_RADIUS`, default 0.12 m). That sphere is an
`AttachedCollisionObject`, and its `touch_links` list every robot link, so the
envelope is ignored against the arm while still blocking the scanner from
driving into the scan object, the supports and the floor.

This split is deliberate. URDF link collisions can only be excluded through
`disable_collisions` entries in the SRDF, which comes from `ur_moveit_config`
and knows nothing about these links — so a scanner with real URDF collision
geometry would be permanently self-colliding unless you patch a third-party
package. `touch_links` travels with the message instead.

If you add links to the scanner, add them to `SCANNER_COLLISION_TOUCH_LINKS`
too. Anything missing from that list is treated as a real collision and will
fail plans. MoveIt logs `has visual geometry but no collision geometry` for
the scanner links on startup; that is expected.
- Gazebo Classic (`ros-humble-ur-simulation-gazebo`) vs. new Gazebo/Ignition
  (`ros-humble-ur-simulation-gz` on some setups) — if your installed UR
  packages use the newer Gazebo, the launch file name and args differ; the
  `--show-args` check in step 5 will tell you which world you're in.
