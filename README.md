# my_arm_with_ball
A ROS 2 and Gazebo project for developing an apple pick-and-place system.

## Included packages

* `my_arm_with_ball_description` - holds SDF descriptions and simulation assets.

* `my_arm_with_ball_gazebo` - holds Gazebo worlds and Gazebo-specific systems.

* `my_arm_with_ball_application` - holds ROS 2 application nodes.

* `my_arm_with_ball_bringup` - holds launch files and bridge configuration.


## Install

The project currently targets the ROS 2 and Gazebo combination installed in the development workspace. For other combinations, follow the official [ROS–Gazebo installation guide](https://gazebosim.org/docs/latest/ros_installation).

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation

   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

    ```bash
    export GZ_VERSION=harmonic
    ```
    Also need to build [`ros_gz`](https://github.com/gazebosim/ros_gz) and [`sdformat_urdf`](https://github.com/ros/sdformat_urdf) from source if binaries are not available for your chosen combination.

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

## Usage

1. Install dependencies

    ```bash
    cd /path/to/my_arm_with_ball
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths . --ignore-src -r -y
    ```

1-1. remove old files
```
rm -rf build install log
```

1. Build the project

    ```bash
    colcon build
    ```

1. Source the workspace

    ```bash
    source install/setup.bash
    ```

1. Launch the simulation

    ```bash
    ros2 launch my_arm_with_ball_bringup simulation.launch.py
    ```

The launch file starts Gazebo, MoveIt `move_group`, the ROS-Gazebo bridge,
robot_state_publisher, the ros2_control controllers, and RViz with the
MotionPlanning panel. Gazebo loads the controller manager through the
`gz_ros2_control` model plugin. MoveIt is started after the arm and gripper
controllers are active. Use `rviz:=false` to omit RViz.

For a machine without a display, run `ros2 launch my_arm_with_ball_bringup
simulation.launch.py headless:=true`. This also disables RViz by default.
The launch sets Gazebo's working directory to the installed bringup package
so the SDF plugin can load its controller YAML through a relative path.

## MoveIt planning

The `my_arm_with_ball_moveit_config` package provides a planning-only MoveIt
demo for the UR5 and RG2. To plan and execute against the Gazebo simulation,
use the integrated `simulation.launch.py` command above. The independent
planning demo remains available without Gazebo:

```bash
colcon build --packages-select my_arm_with_ball_moveit_config --symlink-install
source install/setup.bash
ros2 launch my_arm_with_ball_moveit_config demo.launch.py
```

`demo.launch.py` is the usual entry point. It starts `robot_state_publisher`,
`joint_state_publisher` (an independent home pose, not Gazebo joint states),
MoveGroup, the configured planning scene, and RViz with the MotionPlanning
panel. To run the planning components without RViz:

```bash
ros2 launch my_arm_with_ball_moveit_config demo.launch.py use_rviz:=false
```

The two other launch files are building blocks that `demo.launch.py` includes:

* `move_group.launch.py` starts only the MoveGroup planning node. It loads the
  UR5/RG2 MoveIt configuration and OMPL pipeline, and publishes planning scene
  updates. It intentionally disables trajectory execution. Use this when
  assembling a custom launch setup; that setup must also provide the robot
  description and state topics that MoveGroup monitors. It does not start
  RViz, state publishers, or the demo planning scene loader.

  ```bash
  ros2 launch my_arm_with_ball_moveit_config move_group.launch.py
  ```

* `moveit_rviz.launch.py` starts RViz with the package's MoveIt configuration
  and MotionPlanning display. Use it to attach the planning UI to an already
  running MoveGroup and robot state setup. By itself it does not start
  MoveGroup or publish robot state.

  ```bash
  ros2 launch my_arm_with_ball_moveit_config moveit_rviz.launch.py
  ```

For normal use, start only `demo.launch.py`; launching the two component files
separately is intended for custom integrations and requires their supporting
ROS nodes to be running. The planning-only demo does not start Gazebo or send
trajectories to the robot. The included validation executable can request
plans without executing them:

```bash
ros2 run my_arm_with_ball_moveit_config validate_planning.py
```

## UR5 RG2 keyboard control

The keyboard node sends short `FollowJointTrajectory` action goals to
`arm_trajectory_controller` and reads the latest `/joint_states` before
calculating relative arm targets. It waits up to 120 seconds for the arm and
gripper controllers, the trajectory action server, and a fresh joint state; it
does not send a startup arm reset. The `o/c` gripper commands still use the
`gripper_position_controller` topic. In a second interactive terminal, run:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run my_arm_with_ball_application keyboard_joint_teleop
```

The keyboard node uses its own terminal. Select a joint with the number keys,
then change its target position in 0.05 rad increments:

| Key | Action |
| --- | --- |
| `1` ... `6` | Select UR5 joint 1 ... 6 |
| `a` | Decrease the selected joint target by 0.05 rad |
| `d` | Increase the selected joint target by 0.05 rad |
| `o` | Open the RG2 gripper |
| `c` | Close the RG2 gripper |
| `r` | Move the selected arm joint to 0 rad |
| `x` | Move all UR5 joints to 0 rad; the gripper is unchanged |
| `q` | Quit the keyboard node |

During an arm trajectory, additional arm keys are ignored. The keyboard's
direct action path does not invoke MoveIt's collision planning; use small
motions around the clear starting pose when testing it. RViz/MoveIt plans are
collision checked and use the same arm trajectory controller for execution.

The `joint_state_broadcaster` publishes the measured arm and gripper state on
`/joint_states`. Check the controller and state topics with:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
ros2 topic info /joint_states -v
```

The local `ur5_rg2` model is based on
`https://fuel.gazebosim.org/1.0/anni/models/ur5_rg2/1` and is distributed
under the Creative Commons Attribution 4.0 International license.

If the keyboard node reports that it requires an interactive terminal, start
it separately from a terminal after launching the simulation:

```bash
ros2 run my_arm_with_ball_application keyboard_joint_teleop
```

The `BasicSystem` and `FullSystem` sources in `my_arm_with_ball_gazebo` remain
template Gazebo system scaffolds; keyboard control does not require either
plugin.

# Build after changing the robot model or controllers
```sh
colcon build --packages-select my_arm_with_ball_description my_arm_with_ball_bringup --symlink-install
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py
```
## 각 조인트 위치 참고
https://chatgpt.com/share/6aad354b-b584-83ee-bf4e-08efa79133ac

# 패키지 별로 부분 빌드.
```sh
colcon build --packages-select my_arm_with_ball_application --symlink-install
```

# 코드 스니펫
```sh
# 로컬에 받아온 MODEL SDF 찾기
find ~/.gz -type f | grep -i "model.sdf"
```
