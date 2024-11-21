cmake_minimum_required(VERSION 3.8)
project(dart_flysystem_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  dart_flysystem_hardware
  SHARED
  hardware/dart_flysystem_hardware_servo.cpp
  hardware/dart_flysystem_hardware_imu.cpp
  hardware/linux_pwm.cpp
  hardware/wit_c_sdk.c
)
target_compile_features(dart_flysystem_hardware PUBLIC cxx_std_17)
target_include_directories(dart_flysystem_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/dart_flysystem_hardware>
)
ament_target_dependencies(
  dart_flysystem_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dart_flysystem_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/dart_flysystem_hardware
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/dart_flysystem_hardware
)

install(
  DIRECTORY description/control description/urdf
  DESTINATION share/dart_flysystem_hardware
)

install(TARGETS dart_flysystem_hardware
  EXPORT export_dart_flysystem_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_targets(export_dart_flysystem_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
