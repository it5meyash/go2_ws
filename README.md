# Go2 Quadruped Robot — ROS2 Simulation Workspace

> **Status:** ✅ Full integration working — build, launch, gait, odometry, and walking all validated.
> **Next:** SLAM + Nav2 navigation stack.

## Overview

This workspace provides a complete simulation environment for the **Unitree Go2**
quadruped robot in **Gazebo Harmonic** using **ROS 2 Jazzy**. The robot walks
with the **CHAMP** (ChAracterizing and Modeling Periodic gait) open-source gait
engine, which generates joint trajectories from velocity commands via inverse
kinematics.

The integration required solving ~10 distinct bugs spanning the URDF,
controller configuration, CHAMP's kinematics library, tf2 API mismatches, and
TF broadcast gaps. Every issue has been root-caused, fixed, and validated with
ground-truth measurements — not just "it doesn't crash."

---

## Environment

| Component | Version / Details |
|-----------|-------------------|
| **Host** | Apple M1 Mac (ARM64), Ubuntu 24.04 in UTM |
| **GPU** | virgl (no hardware acceleration, **Gazebo GUI not available**) |
| **ROS 2** | Jazzy Jalisco (`/opt/ros/jazzy`) |
| **Simulator** | Gazebo Harmonic (`ros_gz_sim`, headless mode `-s -r`) |
| **Build System** | colcon (`colcon build`) |
| **Language** | C++17, Python 3 |
| **Controller** | `joint_trajectory_controller/JointTrajectoryController` (position interface) |
| **Gait Engine** | CHAMP (header-only C++ library + `champ_base` ROS2 node wrappers) |

---

## Workspace Structure

```
~/Desktop/ros2_ws2/
├── build/                  # Colcon build artifacts (generated)
├── install/                # Colcon install targets (generated)
├── log/                    # Colcon build logs (generated)
├── src/                    # Source packages (you work here)
│   ├── go2_description/    # Robot model: URDF/xacro, meshes, controller & gait config
│   ├── go2_bringup/        # Launch file, Gazebo bridge config, world files
│   ├── champ/              # Core gait/IK/odometry algorithm (header-only C++ library)
│   ├── champ_base/         # ROS2 wrappers: quadruped_controller, state_estimation, message_relay
│   └── champ_msgs/         # Custom ROS2 message definitions
├── CLAUDE.md               # Claude Code assistant instructions (workspace-specific)
├── FIX_SUMMARY.md          # Summary of state_estimation compilation fixes
├── README.md               # ← You are here
└── .claude/                # Claude Code project configuration
```

### Package Details

#### `go2_description` — The Robot Model

| File / Dir | Purpose |
|------------|---------|
| `xacro/go2_description.xacro` | Main robot description — assembles trunk, IMU, camera, and 4 legs |
| `xacro/const.xacro` | Physical constants: link dimensions, masses, inertias, joint limits |
| `xacro/leg.xacro` | Parametric leg macro — generates hip→thigh→calf→foot chain for FL/FR/RL/RR |
| `xacro/gazebo.xacro` | Gazebo plugin (`gz_ros2_control`) + friction coefficients per link |
| `xacro/materials.xacro` | Visual materials (colors) |
| `meshes/*.dae` | COLLADA mesh files for trunk and leg segments |
| `config/controllers.yaml` | ros2_control config: joint_state_broadcaster + joint_trajectory_controller |
| `config/gait.yaml` | CHAMP gait parameters + links_map/joints_map (critical — see Bug History) |
| `rviz/go2_config.rviz` | RViz2 view configuration |
| `urdf/go2_description.urdf` | Pre-generated URDF (fallback; xacro is primary) |
| `launch/` | Standalone display launch files (RViz-only, no Gazebo) |

#### `go2_bringup` — Launch & Bridge

| File / Dir | Purpose |
|------------|---------|
| `launch/go2_gazebo.launch.xml` | **Primary launch file** — starts Gazebo, spawns robot, loads controllers, starts RViz |
| `config/gazebo_bridge.yaml` | `ros_gz_bridge` topic mapping (Clock, cmd_vel, odom, joint_states) |
| `worlds/warehouse.world` | Alternate world file (warehouse environment) |
| `worlds/empty.sdf` | Default empty world used with `-s -r` flags |

#### `champ` — Gait Library (Header-Only)

| File | Purpose |
|------|---------|
| `kinematics/kinematics.h` | Forward/inverse kinematics for 3-DOF quadruped leg |
| `body_controller/body_controller.h` | Full-body IK — maps Twist → 12 joint positions |
| `leg_controller/leg_controller.h` | Per-leg trajectory generation (swing/stance phases) |
| `leg_controller/phase_generator.h` | Gait phase timing (trot, walk, etc.) |
| `leg_controller/trajectory_planner.h` | Foot trajectory splines |
| `odometry/odometry.h` | Leg-odometry computation from foot contacts |
| `geometry/geometry.h` | 3D rotation/conversion utilities |
| `quadruped_base/` | Base class definitions (joint, leg, robot components) |
| `utils/urdf_loader.h` | URDF parsing utility |
| `bla/` | Basic Linear Algebra — lightweight matrix/vector math |

#### `champ_base` — ROS2 Node Wrappers

| File | Purpose |
|------|---------|
| `src/quadruped_controller.cpp` | Subscribes to `/cmd_vel/smooth`, runs CHAMP IK, publishes joint trajectory |
| `src/state_estimation.cpp` | Leg odometry, IMU orientation, publishes `/odom/raw` + TF transforms |
| `src/message_relay.cpp` | Relays joint states, contacts, IMU between topics |
| `launch/champ_base.launch.py` | Launches all 3 CHAMP nodes with gait config + URDF |
| `config/ekf/` | EKF configs for `robot_localization` (unused currently) |
| `config/velocity_smoother/` | Velocity smoothing config (unused currently) |

#### `champ_msgs` — Custom Messages

| Message | Fields |
|---------|--------|
| `Contacts` | `bool[4]` — foot contact states |
| `ContactsStamped` | Contacts + header |
| `Imu` | Orientation (quaternion), angular velocity, linear acceleration |
| `Joints` | `string[12]` names, `float64[12]` positions/velocities/efforts |
| `PID` | `float64 p, i, d` |
| `Point` | `float64 x, y, z` |
| `PointArray` | `Point[] points` |
| `Pose` | `Point position, float64 yaw` |
| `Velocities` | `float64[12]` — per-joint velocities |

---

## Quick Start

### Prerequisites

```bash
# Install ROS2 Jazzy (if not already)
# https://docs.ros.org/en/jazzy/Installation.html

# Install Gazebo Harmonic
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# Install teleop
sudo apt install ros-jazzy-teleop-twist-keyboard

# Install ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# Install additional deps
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher
```

### Build

```bash
cd ~/Desktop/ros2_ws2
source /opt/ros/jazzy/setup.bash
colcon build
```

✅ Expected: `Summary: 5 packages finished [champ_msgs, champ, go2_description, go2_bringup, champ_base]`

### Run (3 terminals)

**Terminal 1 — Gazebo + Robot + Controllers:**
```bash
cd ~/Desktop/ros2_ws2
source install/setup.bash
ros2 launch go2_bringup go2_gazebo.launch.xml
```
Wait ~10 seconds for Gazebo to stabilize and the control manager to spawn both
controllers. You should see `Successfully switched controllers!` in the output.

**Terminal 2 — CHAMP Gait Controller:**
```bash
cd ~/Desktop/ros2_ws2
source install/setup.bash
ros2 launch champ_base champ_base.launch.py
```
Wait ~5 seconds. The robot is now ready to accept velocity commands.

**Terminal 3 — Teleop:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel/smooth
```

### Teleop Keybindings

| Key | Motion |
|-----|--------|
| `U` | Forward (+x linear) |
| `O` | Backward (-x linear) |
| `I` | Strafe left (+y linear) |
| `,` | Strafe right (-y linear) |
| `J` | Rotate left (+z angular) |
| `L` | Rotate right (-z angular) |
| `K` | **Stop** (zero all) |

> ⚠️ **Always press `K` to stop before finishing a test.** Stale velocity
> commands persist and the robot will keep walking.

### Alternative: Direct Topic Publishing (for testing)

```bash
# Drive forward at 0.3 m/s
ros2 topic pub /cmd_vel/smooth geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  -r 10 &

# Stop after testing
kill %1
ros2 topic pub --once /cmd_vel/smooth geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Data Flow Architecture

```
Teleop / cmd_vel publisher
        │
        ▼
/cmd_vel/smooth  ─────────────────────────────────────────┐
        │                                                  │
        ▼                                                  │
quadruped_controller_node                                  │
  ├─ CHAMP body_controller: Twist → 12 foot positions      │
  ├─ CHAMP leg_controller: foot positions → 12 joint angles│
  └─ Publishes to: /joint_trajectory_controller/joint_trajectory
        │                                                  │
        ▼                                                  │
joint_trajectory_controller (ros2_control)                 │
  └─ Commands 12 joint positions via gz_ros2_control       │
        │                                                  │
        ▼                                                  │
Gazebo Harmonic (physics simulation)                       │
  ├─ Publishes: /world/empty/model/go2/joint_state          │
  └─ Publishes: /model/go2/odometry (ground truth)          │
        │                                                  │
        ▼                                                  │
ros_gz_bridge                                               │
  ├─ Gz JointState   → ROS /joint_states                   │
  └─ Gz Odometry     → ROS /odom (ground truth, unused)    │
        │                                                  │
        ▼                                                  │
/clock  (GZ_TO_ROS)                                        │
/joint_states  (GZ_TO_ROS) ────────────────────────────────┤
        │                                                  │
        ▼                                                  │
message_relay_node                                         │
  ├─ /joint_states → /joint_states/raw                     │
  ├─ Imu relay                                            │
  └─ Contacts relay                                        │
        │                                                  │
        ▼                                                  │
state_estimation_node                                      │
  ├─ Reads /joint_states/raw (positions + velocities)       │
  ├─ CHAMP odometry: foot kinematics → body displacement   │
  ├─ Publishes /odom/raw                                   │
  ├─ Broadcasts TF: odom → base_footprint                  │
  └─ Broadcasts TF: base_footprint → base_link             │
        │                                                  │
        ▼                                                  │
robot_state_publisher                                      │
  └─ Broadcasts TF: base_link → all leg links               │
        │                                                  │
        ▼                                                  │
RViz2 (visualizes full TF tree + robot model)              │
```

### Key Topics

| Topic | Publisher | Type | Purpose |
|-------|-----------|------|---------|
| `/cmd_vel/smooth` | Teleop / user | `Twist` | Velocity command input |
| `/joint_trajectory_controller/joint_trajectory` | `quadruped_controller_node` | `JointTrajectory` | 12-joint position commands |
| `/joint_states` | `ros_gz_bridge` (from Gazebo) | `JointState` | Actual joint positions from physics |
| `/joint_states/raw` | `message_relay_node` | `JointState` | Relayed joint states for odometry |
| `/odom/raw` | `state_estimation_node` | `Odometry` | Leg-based odometry estimate |
| `/odom` | `ros_gz_bridge` (from Gazebo) | `Odometry` | Gazebo ground-truth odometry |
| `/clock` | `ros_gz_bridge` (from Gazebo) | `Clock` | Simulation time (use_sim_time) |

### Key TF Transforms

| Transform | Broadcaster | Purpose |
|-----------|-------------|---------|
| `odom → base_footprint` | `state_estimation_node` | Robot's estimated position in world |
| `base_footprint → base_link` | `state_estimation_node` | Base roll/pitch from IMU |
| `base_link → trunk` | `robot_state_publisher` (URDF) | Fixed joint |
| `trunk → {FL,FR,RL,RR}_hip` | `robot_state_publisher` (URDF) | Leg roots |
| `hip → thigh → calf → foot` | `robot_state_publisher` (URDF) | Leg kinematic chains |

---

## Ground-Truth Validation (Headless)

Since there is no Gazebo GUI on this machine, use these commands to verify
things are working:

### Is the robot physically moving?
```bash
# Before
gz topic -e -t /world/empty/pose/info -n 1 | grep -A6 'name: "go2"'

# Send velocity for 8 seconds...

# After
gz topic -e -t /world/empty/pose/info -n 1 | grep -A6 'name: "go2"'
```
✅ Robot moved if `position.x` changed by > 1.0 m and `position.z` stayed ~0.28–0.30.

### Are joints receiving valid positions?
```bash
ros2 topic echo /joint_trajectory_controller/joint_trajectory --once
```
✅ Valid: 12 joint names, all positions in range ±3.14 rad.
❌ Invalid: values like `1e+29`, `-423040`, `8.9e-41` → IK bug (see Bug History).

### Are all nodes alive?
```bash
ros2 node list
```
Expected after full launch: `/controller_manager`, `/gz_ros_control`,
`/joint_state_broadcaster`, `/joint_trajectory_controller`,
`/robot_state_publisher`, `/ros_gz_bridge`, `/rviz`,
`/quadruped_controller_node`, `/state_estimation_node`, `/message_relay_node`

### Are controllers active?
```bash
ros2 control list_controllers
```
Expected: `joint_state_broadcaster [active]`, `joint_trajectory_controller [active]`

### Is odometry publishing clean values?
```bash
ros2 topic echo /odom/raw --once
```
✅ Valid: `pose.pose.position.x ≈ 0.0` at rest.
❌ Invalid: values like `2.7e+20` → uninitialized state variable.

---

## Gait Configuration

Located at `src/go2_description/config/gait.yaml`. Key tunable parameters:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `gait.nominal_height` | 0.28 m | Standing height of the robot body |
| `gait.swing_height` | 0.04 m | How high feet lift during swing phase |
| `gait.stance_depth` | 0.0 m | How far feet push below ground during stance |
| `gait.stance_duration` | 0.25 s | Duration of ground contact phase |
| `gait.max_linear_velocity_x` | 0.5 m/s | Forward speed limit |
| `gait.max_linear_velocity_y` | 0.25 m/s | Lateral speed limit |
| `gait.max_angular_velocity_z` | 0.7 rad/s | Rotation speed limit |
| `gait.com_x_translation` | 0.0 m | Center of mass offset |
| `gait.knee_orientation` | `">>"` | Knee bending direction (all outward) |
| `gait.pantograph_leg` | false | Leg mechanism type (Go2 uses serial, not pantograph) |
| `gait.odom_scaler` | 1.0 | Odometry correction factor |
| `orientation_from_imu` | false | Use IMU for orientation (instead of foot contacts) |

> ⚠️ **links_map ordering is critical.** The arrays must match the order CHAMP
> expects internally. Wrong order → garbage IK output (see Bug History #4).

---

## Known Bug History (Quick Reference)

For detailed analysis of each bug with root causes and fixes, see
**[BUG_HISTORY.md](BUG_HISTORY.md)**. Quick lookup:

| # | Symptom | Root Cause | Fixed In |
|---|---------|------------|----------|
| 1 | `fatal error: champ/body_controller/body_controller.h` | Missing `champ` package or empty git submodule | `src/champ/` |
| 2 | `champ::champ_lib` target not found | CMake export chain broken; hardcoded include paths work around it | `champ_base/CMakeLists.txt` |
| 3 | Node crashes: "No links/joints config file provided" | YAML scope wrong (`quadruped_controller_node:` not `/**`) + missing `NodeOptions()` | `gait.yaml`, `message_relay.cpp` |
| 4 | Garbage joint values (`1e+29`, `-423040`) | `links_map` array order mismatch, or `l1=0` causing division by zero in IK | `gait.yaml`, `kinematics.h` |
| 5 | Robot won't walk (valid IK, but stuck) | Controller `command_interface` was `effort`, must be `position` | `go2_description.xacro`, `controllers.yaml` |
| 6 | Robot walks, never stops | Stale `/cmd_vel/smooth` from previous session; you must publish explicit zero | Terminal discipline |
| 7 | `/odom/raw` shows `2.7e+20` | `x_pos_`/`y_pos_`/`heading_` uninitialized in state_estimation | `state_estimation.cpp` |
| 8 | `%d` format warning + tf2 compilation errors | `%d` on `long int`; `tf2::Quaternion` API changed (no `.rotate()`, no `operator*`) | `state_estimation.cpp` |
| 9 | Robot walks in Gazebo, appears stationary in RViz | Missing `odom → base_footprint` TF broadcast in state_estimation | `state_estimation.cpp` |

---

## Remaining Work / Next Steps

### Priority 1: SLAM + Nav2 Stack
- [ ] Add sensors to URDF (LIDAR/LDS, depth camera)
- [ ] Configure Gazebo sensor plugins
- [ ] Bridge sensor topics (`ros_gz_bridge`)
- [ ] Configure `slam_toolbox` for online SLAM
- [ ] Configure Nav2 (planner, controller, costmaps)
- [ ] Autonomous navigation with `/cmd_vel` output feeding `/cmd_vel/smooth`

### Priority 2: Gait Tuning
- [ ] Fine-tune `nominal_height`, `swing_height`, `stance_duration` for realism
- [ ] Test lateral strafing and turning stability at various speeds
- [ ] Add velocity smoother integration for smooth acceleration

### Priority 3: Integration Cleanup
- [ ] Convert hardcoded include path in `champ_base/CMakeLists.txt` to proper CMake export
- [ ] Add Gazebo-side odometry plugin for ground-truth comparison
- [ ] Create a single unified launch file (Gazebo + controllers + CHAMP)
- [ ] Add `robot_localization` EKF for fused odometry

### Priority 4: Advanced Features
- [ ] IMU integration (currently `has_imu: false`)
- [ ] Stair/rough terrain worlds
- [ ] Load-carrying simulation (the `load_link` is defined but commented out)
- [ ] Multi-robot scenarios

---

## Useful Debug Commands

```bash
# Check what topics exist
ros2 topic list

# Check topic publishing rate
ros2 topic hz /joint_states
ros2 topic hz /odom/raw

# Check TF tree
ros2 run tf2_tools view_frames
# Then open frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_footprint

# Check Gazebo physics ground truth
gz topic -e -t /world/empty/pose/info -n 1

# Check controller status
ros2 control list_controllers
ros2 control list_hardware_interfaces

# Dump node info
ros2 node info /quadruped_controller_node
ros2 node info /state_estimation_node

# Inspect URDF as processed
ros2 run xacro xacro src/go2_description/xacro/go2_description.xacro | head -200
```

---

## Files Reference

| File | Location | Purpose |
|------|----------|---------|
| README.md | `~/Desktop/ros2_ws2/` | This workspace guide |
| BUG_HISTORY.md | `~/Desktop/ros2_ws2/` | Detailed root-cause analysis of every bug fixed |
| CLAUDE.md | `~/Desktop/ros2_ws2/` | Claude Code assistant instructions |
| FIX_SUMMARY.md | `~/Desktop/ros2_ws2/` | Summary of state_estimation compilation fixes |
| Odometry_Control.txt | `~/Desktop/ros2_ws2/src/` | Notes about TF broadcast fix (walking-in-place) |

---

## Common Pitfalls

1. **Forgetting `source install/setup.bash`** → packages not found, "No executable found"
2. **Not waiting for Gazebo** → controllers spawn before robot exists → errors
3. **Not pressing K to stop** → robot keeps walking from stale Twist messages
4. **Using Gazebo GUI commands** → this machine is headless; use `gz topic -e` instead
5. **Publishing `/cmd_vel` instead of `/cmd_vel/smooth`** → CHAMP never receives the command
6. **Changing `gait.yaml` without rebuilding** → the YAML is in `go2_description` share; changes need `colcon build` (no C++ recompile needed, but install step required)
7. **Mixing ROS distros** — this workspace is Jazzy+Harmonic; `gazebo_ros`, `libgazebo_ros_*`, `ros-humble-*` packages are wrong

---

## Git Info

- **Remote:** `https://github.com/it5meyash/go2_ws.git`
- **Branch:** `main`
- **Last meaningful commits:**
  - `f490de8` — Improve state estimation and update gait configuration
  - `28324df` — Change effort name to position
  - `966f67e` — Change effort controller to joint trajectory controller
  - `1ade478` — Champ integration added

---

*Last updated: 2026-07-03 — Go2/CHAMP integration validated and complete.*