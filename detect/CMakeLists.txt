cmake_minimum_required(VERSION 3.8)
project(detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(info REQUIRED)

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and  license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

set(SOURCES
  src/main.cpp
  src/detect.cpp
)


add_executable(detect_node ${SOURCES})

ament_target_dependencies(detect_node 
rclcpp 
std_msgs 
geometry_msgs 
info
sensor_msgs
image_transport
cv_bridge
)

# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/cameraDriver/DHCamera_for_nx/lib)

target_include_directories(detect_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  # ${CMAKE_CURRENT_SOURCE_DIR}/src/camera$Driver/DHCamera_for_nx/
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  /opt/ros/jazzy/include/cv_bridge
)

target_link_libraries(detect_node
${OpenCV_LIBRARIES}
)

install(TARGETS
detect_node
  DESTINATION lib/${PROJECT_NAME}
)


