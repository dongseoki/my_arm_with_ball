# Copilot instructions for `my_arm_with_ball`

## Build, test, and run

This is a ROS 2 `ament_cmake` workspace containing four packages. Work from
the repository root and source the selected ROS distribution before invoking
`colcon`:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py
```

The Gazebo package selects Garden by default and selects Harmonic when
`GZ_VERSION=harmonic` is exported before configuring/building:

```bash
export GZ_VERSION=harmonic
colcon build
```

For changes limited to SDF/world or other package assets, use a focused build
and symlink install:

```bash
colcon build --packages-select my_arm_with_ball_gazebo --symlink-install
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py
```

Testing is currently lint-only: each package enables
`ament_lint_auto` under `BUILD_TESTING`, and there are no project-specific unit
test files or test executables. Run the full package test/lint pass with:

```bash
colcon test
colcon test-result --verbose
```

Run the test/lint pass for one package with:

```bash
colcon test --packages-select my_arm_with_ball_bringup
colcon test-result --verbose
```

If the workspace configuration is stale after changing package dependencies or
the selected Gazebo version, remove the generated `build/`, `install/`, and
`log/` directories and rebuild. These directories are ignored and are not
source inputs.

## Architecture

- `my_arm_with_ball_description` owns SDF models/assets. Its ament environment
  hooks add the installed `share/.../models` directory to
  `GZ_SIM_RESOURCE_PATH`.
- `my_arm_with_ball_gazebo` owns the Gazebo world and C++ system plugins. The
  CMake file builds the `BasicSystem` and `FullSystem` shared libraries,
  installs `worlds/`, and uses environment hooks to expose both world assets
  and plugin libraries through Gazebo search paths. These two systems are
  currently template/scaffold implementations that log simulation callbacks;
  robot-specific behavior should be added here.
- `my_arm_with_ball_bringup` is the runtime composition layer. Its launch file
  starts `ros_gz_sim` with
  `my_arm_with_ball_gazebo/worlds/apple_pick_place.sdf`, starts
  `ros_gz_bridge` using `config/bridge.yaml`, and optionally starts RViz2 via
  the `rviz` launch argument (default `true`).
- `my_arm_with_ball_application` is reserved for ROS 2 application nodes and
  currently contains only the package scaffold.

The current world includes Gazebo system plugins plus Panda arm, ball, and
basket models fetched from Gazebo Fuel. Runtime ROS/Gazebo integration is
currently limited to the `/clock` bridge in `bridge.yaml`; add additional
topic mappings there when application nodes need simulator data or commands.

## Repository-specific conventions

- Keep package names, `project(...)` names, and installed paths aligned. Assets
  are consumed from the installed package share directory, not directly from
  the source tree at runtime.
- When adding a Gazebo system plugin, declare its class/interface inheritance
  in the header, register it in the `.cc` file with `GZ_ADD_PLUGIN`, add its
  shared-library target to `my_arm_with_ball_gazebo/CMakeLists.txt`, and keep
  the plugin discoverable through `GZ_SIM_SYSTEM_PLUGIN_PATH`.
- Preserve both shell (`*.sh.in`) and machine-readable (`*.dsv.in`) ament
  environment hooks when adding resource or plugin search paths.
- Launch code should resolve package resources with
  `get_package_share_directory(...)` and pass installed paths to included
  launch files and nodes; do not hard-code workspace-relative paths.
- Keep simulation topic translations in `my_arm_with_ball_bringup/config/bridge.yaml`
  and use Gazebo message types on the Gazebo side and ROS message types on the
  ROS side.
- For world/model-only iteration, prefer `--symlink-install`; for C++ plugin,
  CMake, dependency, or Gazebo-version changes, use a normal rebuild and
  re-source `install/setup.bash`.
