# CHAMP Teleop Integration — Full Summary

## Goal
Add velocity-based teleop control to the Go2 ROS 2 Jazzy + Gazebo Harmonic
project, using `chvmp/champ`'s gait/IK controller (not the full nav/SLAM
stack, not `champ_teleop` — keyboard teleop instead).

---

## Packages Pulled In
From the official `ros2` branch of `chvmp/champ` and `chvmp/champ_teleop`:
- `champ` — core gait/IK/odometry algorithm library (header-only)
- `champ_msgs` — custom message types
- `champ_base` — ROS2 node wrappers (`quadruped_controller_node`,
  `state_estimation_node`, `message_relay_node`)
- `champ_teleop` — cloned but ultimately **not used**; replaced with
  `ros-jazzy-teleop-twist-keyboard` to keep the dependency footprint minimal

Explicitly skipped: `champ_navigation`, `champ_gazebo`, `champ_description`,
`champ_config`, `champ_bringup` — all nav/SLAM/ROS1-Gazebo packages, since
the project already has its own `go2_description`/`go2_bringup` and only
needed the controller layer.

---

## Key Learnings

**1. A GitHub repo name and an internal package name can collide.**
`chvmp/champ`'s repo contains a package also named `champ` — easy to
mistake for "already have it" when you've only grabbed `champ_base` and
`champ_msgs`. Always check what packages exist inside a cloned repo, not
just what the repo is named.

**2. Plain `git clone` does not populate submodules.**
`champ/include/champ` is wired as a submodule pointing to a separate repo
(`chvmp/libchamp`). It silently checks out as an empty directory unless you
clone with `--recursive` or run `git submodule update --init --recursive`
afterward. This produced confusing "file not found" header errors that
looked like a path/CMake problem but were actually a missing-content
problem — the directory existed, it was just empty.

**3. ament_cmake's exported-target mechanism can be fragile/inconsistent
across colcon invocations.**
Properly exporting a CMake target (`ament_export_targets`,
`install(TARGETS ... EXPORT ...)`) is the "correct" way to share a
header-only library's include paths with downstream packages. It worked in
an isolated manual `cmake` test but kept failing intermittently inside
colcon builds for reasons never fully root-caused. When a "correct" fix
becomes a time sink with inconsistent results, falling back to a more
direct, deterministic approach (hardcoding the include path) is a
reasonable engineering tradeoff — explicitly logged as technical debt
rather than silently left as a mystery.

**4. A library's own internal headers may use a different include-root
assumption than its consumers.**
`champ_base` includes `<champ/odometry/odometry.h>` (prefixed), but that
header internally includes `<quadruped_base/quadruped_base.h>` (not
prefixed). This meant two separate include directories needed to be added
(`install/champ/include` AND `install/champ/include/champ`), not just one
— a subtlety that only became visible by reading the actual compiler
invocation (`make VERBOSE=1`), not just the error message.

**5. When debugging a stuck build error, get the literal compiler/configure
command, don't keep guessing from symptoms.**
Several rounds of "still fails" were resolved only after extracting the
actual `g++` command line, the actual CMake configure output, and the
literal generated `.cmake` export files — rather than continuing to guess
based on the error text alone. Direct inspection of generated artifacts
(`export_*Export.cmake`, `compile_commands.json`, raw `cmake` runs outside
colcon) consistently cut through ambiguity faster than another clean
rebuild.

**6. Parameter file scope in ROS2 YAML matters — `/**:` exists for a
reason.**
Parameters declared only under one named node section
(`quadruped_controller_node: ros__parameters: ...`) are invisible to other
nodes loading the same file. `links_map`/`joints_map`/`gait.*` needed to be
under `/**:` (wildcard, applies to every node) since `quadruped_controller_node`,
`state_estimation_node`, and `message_relay_node` all independently call
into the same URDF-parsing utility functions and all need the same keys.

**7. CHAMP's `urdf_loader.h` expects a specific, undocumented link-array
order.**
`links_map.<leg>` must be exactly `[hip_link, thigh_link, calf_link,
foot_link]` — four links representing the chain from the first actuated
joint to the foot tip. The root/trunk link is implicit (read via
`model.getRoot()`), never listed. Getting this wrong doesn't crash — it
silently computes wrong (zero or tiny) leg-segment lengths, which is far
more dangerous than a crash because it fails downstream, far from the
actual mistake.

**8. "No crash" does not mean "no bug." Uninitialized memory is a recurring
failure mode in this codebase.**
Three separate uninitialized-variable bugs were found in this one small
codebase, all producing wildly out-of-range floats (`1e+29`, `-423040.0`,
etc.) instead of crashing outright:
   - `kinematics.h`'s `inverse()`: an early `return` on an unreachable-target
     check left `upper_leg_joint`/`lower_leg_joint` (passed by reference)
     completely unwritten, leaving raw stack garbage. The `isnan()` safety
     check in the caller didn't catch it because the result wasn't strictly
     NaN (it could be `inf` or simply never written).
   - `l1`/`l2` computed as exactly `0.0` (from the link-order bug in point 7)
     then used as a divisor in `acosf(x / l1)` — produced `inf`, not `NaN`,
     slipping past the `isnan()` guard mentioned above.
   - `state_estimation.cpp`'s `x_pos_`/`y_pos_`/`heading_` member floats: never
     initialized in the constructor, immediately corrupted on the first
     `+=` accumulation.
   This is the single most important lesson from this whole integration:
   **always explicitly verify actual numeric output, never just "did it
   crash or not."** A clean launch with no errors can still be silently
   producing garbage.

**9. Forgetting `NodeOptions` parameter flags on one node out of several
nearly-identical nodes is an easy, hard-to-spot bug.**
`message_relay_node`'s constructor was missing
`.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)`
— present on the other two nodes, absent here. This meant it couldn't see
any parameters from the YAML file at all, despite the file being loaded
without error. Comparing nearly-identical constructors side by side caught
this; it would never have been found by reading one file in isolation.

**10. A controller's command message type dictates the
controller_manager controller type — not the other way around.**
`champ_base`'s nodes publish `trajectory_msgs/JointTrajectory`. The
original project used `effort_controllers/JointGroupEffortController`,
which expects `Float64MultiArray`, not `JointTrajectory`. This forced a
switch to `joint_trajectory_controller/JointTrajectoryController`
(position-based), which was already an outstanding TODO in the project
before CHAMP was even introduced — CHAMP integration just made it
mandatory rather than optional.

**11. CHAMP's `ros2` branch has at least one missing feature, not just bugs.**
`state_estimation_node` creates a `tf2_ros::TransformBroadcaster` but never
actually calls `sendTransform()` anywhere — `odom`/`base_footprint` frames
are simply never broadcast. In the original ROS1 architecture, this was
likely handled by a separate `robot_localization`/`ekf` node consuming
CHAMP's topic outputs; that piece was never ported/replaced when this
branch was created. This is a real gap, not something fixable by config
alone — would require either writing a custom TF broadcaster node or
pulling in `robot_localization`.

**12. Always verify against ground truth, not the system's own self-reported
state.**
CHAMP's own `/odom/raw` and the bridged `/odom` topic were both unreliable
(one had a real bug, the other had no publisher at all). The
`gz topic -e -t /world/empty/pose/info` command — querying Gazebo's physics
engine directly — was the only reliable way to confirm whether the robot
was actually translating in the simulated world, independent of any of the
ROS-side software that might itself be buggy.

---

## All Bugs Found & Fixed (chronological)

| # | Bug | Symptom | Fix |
|---|-----|---------|-----|
| 1 | Only `champ_base`/`champ_msgs` cloned, missing `champ` package | `fatal error: champ/body_controller/body_controller.h: No such file` | Cloned `champ` package too |
| 2 | `champ/include/champ` is an uninitialized git submodule | Same header errors persisted after fix #1 | `git submodule update --init --recursive` |
| 3 | `champ`'s CMake export target unreliable under colcon | `Target "champ::champ_lib" ... not found` (intermittent) | Hardcoded `install/champ/include` path directly into `champ_base/CMakeLists.txt`, bypassing target export entirely |
| 4 | libchamp's internal headers use unprefixed includes | `fatal error: quadruped_base/quadruped_base.h: No such file` (after fix #3) | Added second include root: `install/champ/include/champ` |
| 5 | `champ_base` launch file misnamed (`champ_bash.launch.py`) | `file ... was not found in the share directory` | Renamed to `champ_base.launch.py` |
| 6 | `urdf` parameter never populated | `Failed to parse urdf string` / `XML_ERROR_EMPTY_DOCUMENT` | Generated URDF string via Python `xacro.process_file()` in the launch file, passed as a parameter |
| 7 | `links_map`/`joints_map` declared only under one node's YAML scope | `No links/joints config file provided` (crash) | Moved shared parameters under `/**:` wildcard scope |
| 8 | `links_map.<leg>` array order/contents wrong (missing calf link, included trunk) | IK produced garbage joint values (`1e+29`, etc.) | Corrected array to `[hip, thigh, calf, foot]` links exactly, no trunk/root in the array |
| 9 | `message_relay_node` missing `NodeOptions` parameter-declaration flags | `No joints config file provided` (crash, even after fix #7) | Added matching `NodeOptions()` to its constructor |
| 10 | `state_estimation.cpp`'s `x_pos_`/`y_pos_`/`heading_` never initialized | `/odom/raw` showed garbage position (`2.7e+20`) | Explicitly zero-initialized in constructor |
| 11 | Controller interface mismatch (`effort_controller` vs `JointTrajectory`) | N/A (caught proactively before launch) | Switched to `joint_trajectory_controller/JointTrajectoryController` |
| 12 | `state_estimation_node` never calls `sendTransform()` | `odom`/`world` TF frames don't exist | **Not yet fixed** — see Remaining Work |
| 13 | Robot's legs cycle but body doesn't translate | Ground-truth Gazebo pose unchanged after teleop | **RESOLVED** — see below |
| 14 | `go2_gazebo.launch.xml` spawner still referenced old `effort_controller` | `Failed loading controller effort_controller` / `'type' param was not defined` | Updated spawner `args` to `joint_trajectory_controller` |
| 15 | URDF's `<ros2_control>` block still declared `effort` as the only command interface per joint | `Unable to activate controller ... command interface 'FL_hip_joint/position' is not available` | Changed all 12 `<command_interface name="effort"/>` to `<command_interface name="position"/>` in `go2_description.xacro` |

**Bug #13 resolution (the real root cause):** the initial suspicion was a
gait-phase-generator bug (`has_swung_` startup gating in
`phase_generator.h`). That theory turned out to be wrong. The actual cause
was upstream: bugs #14 and #15. With the controller switch
(`effort_controller` → `joint_trajectory_controller`) only half-applied —
`controllers.yaml` and `gait.yaml` had been updated, but the launch file's
spawner arguments and the URDF's `<ros2_control>` hardware interface
declarations had not — the robot had **no active position controller** at
all during every previous walking test. `gz_ros_control` only exposed an
`effort` command interface per joint (leftover from the original
effort-based setup), so `joint_trajectory_controller` could load and
configure but failed to *activate* ("command interface ... not available").
This meant the robot had zero joint stiffness and was passively collapsing
under gravity (observed `z` height silently dropping to under 1cm in one
test) while CHAMP's gait logic was computing perfectly correct joint
trajectories that were never actually being applied to the simulated
joints. Once both gaps were closed — spawner arguments updated, and all 12
`<command_interface>` entries changed from `effort` to `position` — the
robot walked correctly on the very first test (confirmed via
`gz topic -e -t /world/empty/pose/info`, ground-truth physics position
moving steadily forward with stable standing height, both with sustained
`topic pub` commands and with real `teleop_twist_keyboard` input).

**Learning to add to the list above:** when a multi-file config change
(switching controller types) is only partially applied, the failure mode
can be silent and misleading — the system appeared to "almost work" (legs
visibly cycling, correct IK output, no crashes) while the actual blocking
issue was several layers removed from where the symptom was visible. The
fix was found by reading the *full* launch log line by line rather than
just checking exit codes, which surfaced the `effort_controller` spawner
failure that had been silently happening on every single previous launch.

---

## Current Status
✅ Full build chain compiles cleanly (`champ`, `champ_msgs`, `champ_base`,
`go2_description`, `go2_bringup`)
✅ All three `champ_base` nodes launch and stay alive
✅ `cmd_vel/smooth` → `quadruped_controller_node` → IK → `joint_trajectory_controller`
pipeline confirmed working with real (non-garbage) joint values
✅ Legs visibly animate through a gait cycle in RViz in response to teleop input
✅ `joint_trajectory_controller` correctly activates with a real `position`
command interface (after URDF + launch file fixes)
✅ **Robot physically walks** — confirmed via Gazebo ground-truth pose
(`gz topic -e -t /world/empty/pose/info`) translating steadily forward
while maintaining stable standing height, with both sustained `topic pub`
commands and real `teleop_twist_keyboard` input
🟡 TF broadcasting (`odom`→`base_footprint`) still not implemented —
optional for now, needed later for navigation/SLAM
🟡 No native Gazebo odometry plugin — `/odom` topic bridge exists but has
no publisher; not required for teleop, ground-truth verified via direct
`gz topic` queries instead

---

## Remaining Work

1. **Validate turning/strafing**, not just forward motion. Test angular
   velocity (`z`) and lateral (`y`) commands via teleop, confirm the gait
   handles all DOF correctly, not just straight-line forward walking.

2. **Tune gait parameters for real, stable walking** — `nominal_height`
   (currently `0.28`), `swing_height`, `stance_duration`,
   `max_linear_velocity_*` were rough first-guess values. Now that the
   robot actually walks, observe behavior (stability, speed, foot
   slip/scuffing) and adjust.

3. **TF broadcasting is missing entirely.** `state_estimation_node` never
   calls `sendTransform()`. Needed eventually for SLAM/navigation (being
   handled separately) — not required for basic teleop driving.

4. **No Gazebo-side odometry plugin.** `gazebo_bridge.yaml` bridges
   `/model/go2/odometry`, but nothing publishes to that Gazebo topic.
   Worth adding later for a reliable in-ROS odometry source instead of
   relying on manual `gz topic` ground-truth queries.

5. **Hardcoded absolute path in `champ_base/CMakeLists.txt`** (from the
   `champ::champ_lib` CMake export workaround) — same category of issue as
   the pre-existing `gazebo.xacro` hardcoded path TODO. Not portable to
   another machine/user. Low priority.

6. **Confirm the debug `printf` in `kinematics.h` was fully reverted** and
   didn't survive any later rebuild.

7. **Document the final, working launch sequence** as a single combined
   launch file if this gets used repeatedly, instead of manually running
   `go2_bringup`'s and `champ_base`'s launch files in two separate
   terminals every time.