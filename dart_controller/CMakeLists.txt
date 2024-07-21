cmake_minimum_required(VERSION 3.18)
project(dart_controller)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(info REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add node_lvgl_ui target
include_directories(include)
add_executable(node_lvgl_ui src/node_lvgl_ui.cpp src/get_ip.cpp src/dart_config.cpp)
ament_target_dependencies(node_lvgl_ui rclcpp std_msgs info std_srvs sensor_msgs cv_bridge)

# Add Lvgl support
add_subdirectory(lvgl)
add_subdirectory(lv_drivers)

target_include_directories(lvgl PUBLIC include/lvgl lv_drivers)
target_include_directories(lv_drivers PUBLIC lvgl)

target_include_directories(node_lvgl_ui PUBLIC ${PROJECT_SOURCE_DIR} include src/generated src/custom src/generated/guider_customer_fonts src/generated/guider_fonts src/generated/images ${OpenCV_INCLUDE_DIRS})

# 递归添加子目录${PROJECT_SOURCE_DIR}/generated/*
FILE(GLOB_RECURSE SOURCES src/custom/*.c src/generated/*.c /src/dart_config.cpp)
target_sources(node_lvgl_ui PUBLIC ${SOURCES})
target_link_libraries(node_lvgl_ui lvgl lv_drivers ${OpenCV_LIBS})

# Add node_can_agent target
add_executable(node_can_agent src/node_can_agent.cpp src/dart_config.cpp)
ament_target_dependencies(node_can_agent rclcpp std_msgs info)

# Add libsockcancpp support
add_subdirectory(libsockcancpp)
target_include_directories(node_can_agent PUBLIC libsockcancpp/include libsockcancpp/include/exceptions include)
target_link_libraries(node_can_agent libsocketcancpp)

# Add node_dart_config target
add_executable(node_dart_config src/node_dart_config.cpp src/dart_config.cpp)
target_include_directories(node_dart_config PUBLIC include)

# yaml lib
find_package(yaml-cpp REQUIRED)
target_link_libraries(node_dart_config yaml-cpp)
target_link_libraries(node_can_agent yaml-cpp)

# target_link_libraries(node_can_agent yaml-cpp)
ament_target_dependencies(node_dart_config rclcpp std_msgs info std_srvs)

# Set the path to the YAML file relative to the ROS 2 install directory
set(YAML_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/dart_config.yaml")

# Pass the YAML path as a definition with double quotes
add_definitions(-DYAML_PATH=\"${YAML_PATH}\")

install(TARGETS
    node_lvgl_ui
    node_can_agent
    node_dart_config
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# Ensure compile_commands.json is generated
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)