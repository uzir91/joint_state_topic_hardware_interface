cmake_minimum_required(VERSION 3.16)
project(joint_state_topic_hardware_interface CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(
  ${PROJECT_NAME}
  src/joint_state_topic_hardware_interface.cpp
  src/humble_system_interface_compat.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  # The large upstream gmock test uses post-Humble ResourceManager,
  # typed-handle and introspection APIs. Keep the source in the package, but do
  # not compile that test on Humble. The end-to-end launch tests below exercise
  # the actual Humble controller_manager and plugin.

  find_package(ament_cmake_ros REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(std_msgs REQUIRED)

  add_executable(
    jtc_client_position
    test/rrr/jtc_client_position.cpp
  )
  ament_target_dependencies(
    jtc_client_position
    control_msgs
    rclcpp
    rclcpp_action
  )

  add_executable(
    test_gripper
    test/gripper/test_gripper.cpp
  )
  ament_target_dependencies(
    test_gripper
    rclcpp
    std_msgs
  )

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(
    test/rrr/position.test.py
  )

  add_ros_isolated_launch_test(
    test/gripper/position.test.py
  )
endif()

pluginlib_export_plugin_description_file(
  hardware_interface
  joint_state_topic_hardware_interface_plugin_description.xml
)

ament_export_targets(
  ${PROJECT_NAME}Targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
