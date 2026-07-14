cmake_minimum_required(VERSION 3.8)
project(cm_topic_hardware_component CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  hardware_interface
  pal_statistics_msgs
  pluginlib
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
  src/cm_topic_hardware_component.cpp
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

# The upstream test uses post-Humble ResourceManager and introspection APIs.
# Backport the test separately after the plugin itself builds on Humble.
if(BUILD_TESTING)
  message(STATUS
    "Skipping upstream cm_topic_hardware_component test on ROS 2 Humble")
endif()

pluginlib_export_plugin_description_file(
  hardware_interface
  cm_topic_hardware_component_plugin_description.xml
)

ament_export_targets(
  ${PROJECT_NAME}Targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
