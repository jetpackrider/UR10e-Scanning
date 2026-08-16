# UR10e arc-scan demo in Gazebo — workspace setup (Ubuntu 22.04, ROS2 Humble)

Two packages here:
- **`arm_scan_demo`** — the arc-scan node (`move_group_interface.py`) plus the
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

## 2. Install MoveIt2 (includes moveit_py), and the UR ROS2 stack
```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ur \
  ros-humble-ur-simulation-gazebo \
  python3-pip

pip3 install pyquaternion numpy
```
`moveit_py` and `moveit_configs_utils` are pulled in automatically as
dependencies of `ros-humble-moveit` — there's no separate
`ros-humble-moveit-py` package on Humble's apt repos.
If `ros-humble-ur-simulation-gazebo` isn't found for your exact point
release, build it from source instead:
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
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
`move_group` + RViz (so you can watch it), and then — 15 seconds later, once
controllers are up — our scan node, which publishes the target marker, the
candidate `PoseArray`, prompts for Enter, and on Enter sweeps the tool
through the arc around the object.

In RViz, add displays for the `poses` (PoseArray) and `visualization_marker`
(Marker) topics, on the `base_link` frame, to see the planned arc and target
before/while it executes.

## Things you'll likely need to tune
- **`object_position`** in `move_group_interface.py` (currently
  `[0.6, 0.0, 0.4]`, in the `base_link` frame) — put it wherever your actual
  object is relative to the robot base.
- **The tool geometry** in `ur10e_with_tool.urdf.xacro` — currently a plain
  grey cylinder as a placeholder.
- **`radius`/`offset_angle`/`no_of_points`** in `my_go_to_pose_goal()` if the
  arc is too tight/wide or clips the robot's own base.
- Gazebo Classic (`ros-humble-ur-simulation-gazebo`) vs. new Gazebo/Ignition
  (`ros-humble-ur-simulation-gz` on some setups) — if your installed UR
  packages use the newer Gazebo, the launch file name and args differ; the
  `--show-args` check in step 5 will tell you which world you're in.
