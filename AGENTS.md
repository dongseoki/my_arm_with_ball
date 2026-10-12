# 사용자 공통 요청 사항
* 문의 하는 작업에 대해서 코드 수정을 하지 말것.(ex) 이것은 왜이래?, 이코드는 무슨 의미야? ) 
* 명시적인 구현, 코드수정 요청이 있을 경우만 코드를 수정할것.

# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 `ament_cmake` workspace with four packages. `my_arm_with_ball_description/` holds the UR5/RG2 SDF and Xacro models, meshes, and model consistency test. `my_arm_with_ball_gazebo/` contains the Gazebo world, C++ system plugins, and resource hooks. `my_arm_with_ball_bringup/` contains the simulation launch file and controller/bridge YAML. `my_arm_with_ball_application/` contains the keyboard teleoperation node. Design notes and validation steps live in `adr/`, `analysis/`, and `docs/`. Treat `build/`, `install/`, and `log/` as generated output.

## Build, Test, and Development Commands

Run commands from the repository root after sourcing your ROS distribution:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py
```

`rosdep` installs package dependencies; `colcon` builds all packages. For model or launch changes, use `colcon build --packages-select my_arm_with_ball_description my_arm_with_ball_bringup --symlink-install`. Add `headless:=true` to the launch command when no display is available. Run teleoperation in a separate terminal with `ros2 run my_arm_with_ball_application keyboard_joint_teleop`.

## Coding Style & Naming Conventions

Follow the existing four-space indentation in Python and C++, and keep Python modules and launch files in `snake_case` (for example, `simulation.launch.py`). Match ROS package names, CMake project names, and installed resource paths. Resolve launch assets through package share paths; keep bridge mappings and controller settings in `my_arm_with_ball_bringup/config/`. Preserve paired `.sh.in` and `.dsv.in` environment hooks when changing Gazebo search paths. Use the configured `ament_lint_auto` checks; no separate formatter configuration is committed.

## Testing Guidelines

Run `colcon test` followed by `colcon test-result --verbose`. For a focused check, add `--packages-select <package>` to `colcon test`. The description package also runs `test/test_model_consistency.py`, which checks SDF/Xacro geometry, joints, and control interfaces; name new Python tests `test_*.py`. After changes to launch, controllers, or simulation assets, smoke-test the launch and use `docs/control_validation_checklist.md` for controller checks.

## Commit & Pull Request Guidelines

Recent commits mix short imperative subjects with `feat:` and `docs:` prefixes; use a concise subject that says what changed. In pull requests, identify affected packages, describe behavior changes, list build/test commands and results, and include screenshots or logs when simulation behavior or visuals change. Link a related issue or design note when applicable.
