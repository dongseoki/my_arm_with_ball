cmake_minimum_required(VERSION 3.5)

project(my_arm_with_ball_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME ur5_rg2_model_consistency
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_model_consistency.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/models/ur5_rg2/model.sdf"
            "${CMAKE_CURRENT_SOURCE_DIR}/urdf/ur5_rg2.urdf.xacro"
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
