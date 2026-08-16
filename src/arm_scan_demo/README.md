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

In RViz, add displays on the `base_link` frame for `/poses` (PoseArray),
`/visualization_marker` (Marker — sphere, floor, pedestals) and
`/visualization_marker_array` (MarkerArray — waypoint dots and orientation
axes) to see the scan pattern before/while it executes.

## Things you'll likely need to tune
All of these are constants at the top of `move_group_interface.py`:
- **`SPHERE_CENTER`** (currently `[0.55, 0.0, 0.45]`, in the `base_link`
  frame) and **`SPHERE_RADIUS`** — put these where your actual object is.
- **`SCAN_RADIUS`** — distance from the object centre that the tool tip is
  held at. Must clear `SPHERE_RADIUS + TOOL_COLLISION_SPHERE_RADIUS` or every
  waypoint starts in collision.
- **`NUM_POINTS`**, **`SCAN_PATTERN`** (`full_sphere` / `ring` / `bands`) and
  **`ELEVATIONS_DEG`** (used only by `bands`) if the coverage is too
  sparse/dense or clips the robot's own base.
- **`MAX_VELOCITY_SCALING`** / **`MAX_ACCELERATION_SCALING`** — both 0.15.
- **The tool geometry** in `ur10e_with_tool.urdf.xacro` — currently a plain
  grey cylinder as a placeholder. If you change its length, move
  `tool_tip_joint`'s origin to match, and keep the geometry offset so it grows
  outward from `tool0` instead of back into the wrist.
- Gazebo Classic (`ros-humble-ur-simulation-gazebo`) vs. new Gazebo/Ignition
  (`ros-humble-ur-simulation-gz` on some setups) — if your installed UR
  packages use the newer Gazebo, the launch file name and args differ; the
  `--show-args` check in step 5 will tell you which world you're in.
