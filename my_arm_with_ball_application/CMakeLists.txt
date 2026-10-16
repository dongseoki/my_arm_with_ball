cmake_minimum_required(VERSION 3.5)

project(my_arm_with_ball_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(pick_and_place_test src/pick_and_place_test.cpp)
target_compile_features(pick_and_place_test PRIVATE cxx_std_17)
ament_target_dependencies(pick_and_place_test
  rclcpp rclcpp_action action_msgs controller_manager_msgs control_msgs
  geometry_msgs moveit_msgs moveit_task_constructor_core
  moveit_task_constructor_msgs sensor_msgs shape_msgs std_msgs std_srvs)
install(TARGETS pick_and_place_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME keyboard_joint_teleop
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_keyboard_joint_teleop.py"
  )
  add_test(
    NAME grasp_scene
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_grasp_scene.py"
  )
  add_test(
    NAME grasp_logic
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_grasp_logic.py"
  )
  add_executable(test_place_stability test/test_place_stability.cpp)
  target_compile_features(test_place_stability PRIVATE cxx_std_17)
  add_test(NAME place_stability COMMAND test_place_stability)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
    scripts/keyboard_joint_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_joint_teleop
)

install(PROGRAMS scripts/grasp_test.py DESTINATION lib/${PROJECT_NAME} RENAME grasp_test)
install(FILES scripts/grasp_scene.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/grasp_logic.py DESTINATION lib/${PROJECT_NAME})

ament_package()
