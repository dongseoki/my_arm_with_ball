cmake_minimum_required(VERSION 3.5)

project(my_arm_with_ball_bringup)

find_package(ament_cmake REQUIRED)
find_package(my_arm_with_ball_application REQUIRED)
find_package(my_arm_with_ball_description REQUIRED)
find_package(my_arm_with_ball_gazebo REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(
    NAME test_ros2_control_configuration
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s test -p test_ros2_control_configuration.py
  )
  set_tests_properties(test_ros2_control_configuration PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
