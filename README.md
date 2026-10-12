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

The launch file starts Gazebo, RViz, the ROS-Gazebo bridge,
robot_state_publisher, and the two ros2_control controller spawners. Gazebo
loads the controller manager through the `gz_ros2_control` model plugin.
RViz shows the robot model using `/robot_description` and `/joint_states`.
Keyboard teleoperation is run separately. Use `rviz:=false` to omit RViz.

For a machine without a display, run `ros2 launch my_arm_with_ball_bringup
simulation.launch.py headless:=true`. This also disables RViz by default.
The launch sets Gazebo's working directory to the installed bringup package
so the SDF plugin can load its controller YAML through a relative path.

## UR5 RG2 keyboard control

The keyboard node sends the six UR5 target positions to the ros2_control arm
controller. It waits up to 120 seconds for `arm_position_controller` to become
active before sending its initial zero-position target. The `o/c` gripper
commands still use the Gazebo bridge. In a second terminal, run:

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
| `r` | Reset the selected joint to its initial target |
| `x` | Reset all UR5 joints and the gripper |
| `q` | Quit the keyboard node |

The arm controller accepts six target positions in joint order
`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`,
`wrist_2_joint`, `wrist_3_joint`. To test the arm controller directly:

```bash
ros2 topic pub --once /arm_position_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

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
