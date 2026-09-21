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

The launch file starts Gazebo, the ROS-Gazebo bridge, and RViz2. Keyboard
teleoperation is disabled by default because it requires a dedicated
interactive terminal:

```bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py rviz:=false
```

## UR5 RG2 keyboard control

In a second terminal, run the keyboard node after the simulation is started:

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

The node clamps commands to the UR5 and RG2 joint limits and publishes
`std_msgs/msg/Float64` on `/ur5_<joint_name>/cmd_pos` and
`/rg2_finger_joint{1,2}/cmd_pos`. The bridge forwards each topic to the
corresponding Gazebo command topic:

```bash
ros2 topic echo /ur5_wrist_3_joint/cmd_pos
gz topic -e -t /model/ur5_rg2/joint/wrist_3_joint/0/cmd_pos
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

# sdf만 수정한경우
```sh
colcon build --packages-select my_arm_with_ball_gazebo --symlink-install
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py
```

# gazebo 로봇팔 테스트 명령어
```위아래로 고개 흔들기
dslee@dslee-To-Be-Filled-By-O-E-M:~/workspace/my_arm_with_ball$ gz topic -t /model/ur5_rg2/joint/wrist_3_joint/0/cmd_pos   -m gz.msgs.Double   -p 'data: -2'

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