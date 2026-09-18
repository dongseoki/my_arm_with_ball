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
    rosdep install --from-paths src --ignore-src -i -y
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

The initial world contains only a ground plane and verifies the Gazebo, ROS 2, bridge, and RViz2 connection. The `BasicSystem` and `FullSystem` sources in `my_arm_with_ball_gazebo` are retained as the official template's Gazebo system scaffold; they will be replaced or extended when robot-specific simulation behavior is added.
