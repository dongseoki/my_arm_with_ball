cmake_minimum_required(VERSION 3.5)

project(my_arm_with_ball_application)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME keyboard_joint_teleop
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_keyboard_joint_teleop.py"
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
    scripts/keyboard_joint_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_joint_teleop
)

ament_package()
