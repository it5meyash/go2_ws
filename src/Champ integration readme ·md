# CHAMP Integration — Build Issues & Fixes

## Context
Integrating `chvmp/champ`'s controller (gait/IK + state estimation) into the
Go2 ROS 2 Jazzy + Gazebo Harmonic project, for teleop control. Goal was to
take only `champ_base` + `champ_msgs` (+ teleop), without the navigation/SLAM
stack or ROS1 Gazebo plugins.

Packages used, all from the `ros2` branch of the official repos:
- `chvmp/champ` → `champ` (core algorithm library), `champ_base`, `champ_msgs`
- `chvmp/champ_teleop` → `champ_teleop`

---

## Issue 1 — `champ_base` can't find `champ/...` headers at all

**Symptom**
```
fatal error: champ/body_controller/body_controller.h: No such file or directory
fatal error: champ/odometry/odometry.h: No such file or directory
fatal error: champ/utils/urdf_loader.h: No such file or directory
```

**Cause**
Initially cloned only `champ_base` and `champ_msgs` from the repo. These
packages don't contain the actual gait/IK/odometry implementation — that
lives in a separate package, also confusingly named `champ`, in the same
repo (`chvmp/champ/champ/`). It had been skipped entirely.

**Fix**
Cloned the `champ` package alongside `champ_base`/`champ_msgs`:
```bash
git clone -b ros2 https://github.com/chvmp/champ.git champ_tmp
cp -r champ_tmp/champ ~/Desktop/ros2_ws2/src/
```

---

## Issue 2 — `champ` package "builds" but headers are still missing

**Symptom**
Same fatal errors as Issue 1, persisting even after `champ` was added and
reported `Finished` by colcon.

**Cause**
`champ/include/champ` is a **git submodule** pointing to a separate repo,
`chvmp/libchamp`. A plain `git clone` never populates submodule contents —
the directory exists but is empty. Confirmed via:
```bash
cat /tmp/champ_check/.gitmodules
# [submodule "champ/include/champ"]
#   path = champ/include/champ
#   url = https://github.com/chvmp/libchamp
```

**Fix**
```bash
cd <champ clone>
git submodule update --init --recursive
```
This populated `champ/include/champ/` with the real headers
(`body_controller/`, `odometry/`, `quadruped_base/`, `kinematics/`, etc.),
then the folder was re-copied into the workspace.

---

## Issue 3 — CMake target `champ::champ_lib` "not found" during configure

**Symptom**
```
CMake Error: Target "quadruped_controller" links to: champ::champ_lib
but the target was not found.
```
Happened intermittently even after `champ`'s `CMakeLists.txt` was rewritten
to properly export an `ament_cmake` target (`ament_export_targets`,
`install(TARGETS ... EXPORT export_champ)`). Manually running `cmake`
directly (outside colcon) proved the export *could* work — `find_package
(champ)` succeeded and the compiled `-isystem` include path was correct in
that isolated test — but colcon builds kept failing the same way regardless
of clean rebuilds, sequential execution, or full workspace nukes.

**Cause**
Not fully root-caused — suspected interaction between colcon's environment
setup and the ament_cmake exported-target mechanism for this particular
package, possibly related to the leftover nested `.git` from the submodule
checkout interfering with install/config generation. Rather than keep
debugging an intermittent CMake export chain, switched to a more direct,
deterministic approach.

**Fix (final)**
Bypassed the CMake target export entirely. Since `champ` is a header-only
library, only the include path is actually needed — no linking required.
In `champ_base/CMakeLists.txt`:
```cmake
include_directories(
  include
  /home/yash/Desktop/ros2_ws2/install/champ/include
  /home/yash/Desktop/ros2_ws2/install/champ/include/champ
)
```
and removed all `champ::champ_lib` references from `target_link_libraries`
calls (nothing to link against — it's header-only).

⚠️ **Known limitation:** this hardcodes an absolute path, same category of
issue as the existing `gazebo.xacro` hardcoded-path TODO in the main project.
Not portable to another machine/user as-is. Revisit later with
`$(find champ)`-style resolution or fixing the underlying export properly.

---

## Issue 4 — Second wave of missing headers, after Issue 1–3 were fixed

**Symptom**
```
champ/odometry/odometry.h:29:10: fatal error: quadruped_base/quadruped_base.h: No such file or directory
champ/body_controller/body_controller.h:31:10: fatal error: geometry/geometry.h: No such file or directory
```

**Cause**
`libchamp`'s own internal headers `#include` each other **without** the
`champ/` prefix (e.g. `#include <quadruped_base/quadruped_base.h>` instead
of `#include <champ/quadruped_base/quadruped_base.h>`). This means the
include root needs to be `install/champ/include/champ` itself, in addition
to `install/champ/include` (which is what `champ_base`'s own files need,
since *they* `#include <champ/odometry/odometry.h>` with the prefix).

**Fix**
Added both include roots (already shown combined in Issue 3's fix above):
```cmake
include_directories(
  include
  /home/yash/Desktop/ros2_ws2/install/champ/include        # for champ_base's own #include <champ/...>
  /home/yash/Desktop/ros2_ws2/install/champ/include/champ  # for libchamp's internal #include <quadruped_base/...>
)
```

---

## Result
`champ`, `champ_msgs`, `champ_base`, `champ_teleop` all build cleanly:
```bash
cd ~/Desktop/ros2_ws2
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Still Outstanding
1. **Hardcoded path** in `champ_base/CMakeLists.txt` (Issue 3 fix) — not
   portable across machines/users.
2. **`champ_base` ships no launch files** — a custom `champ_base.launch.py`
   was written from scratch, loading parameters from
   `go2_description/config/gait.yaml`.
3. **`links_map` / `joints_map` parameter schema** had to be reverse-engineered
   from `urdf_loader.h` source directly (no example config shipped with this
   branch's `champ_base`) — confirmed to be flat 4-element / 3-element string
   arrays per leg, not nested YAML dictionaries.
4. **Controller interface mismatch**: `champ_base` publishes
   `trajectory_msgs/JointTrajectory` by default — the project's original
   `effort_controller` (effort-based) needed to be switched to a
   `joint_trajectory_controller/JointTrajectoryController` (position-based)
   to consume this directly.
5. **`champ_teleop`'s actual output topic** vs. `champ_base`'s expected input
   (`cmd_vel/smooth`, not raw `/cmd_vel`) still needs confirmation/wiring.