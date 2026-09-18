cmake_minimum_required(VERSION 3.5)

project(my_arm_with_ball_application)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/keyboard_joint_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_joint_teleop
)

ament_package()
