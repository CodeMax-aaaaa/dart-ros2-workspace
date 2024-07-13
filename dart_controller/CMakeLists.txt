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

# Add node_lvgl_ui target
include_directories(include)
add_executable(node_lvgl_ui src/node_lvgl_ui.cpp src/get_ip.cpp)
ament_target_dependencies(node_lvgl_ui rclcpp std_msgs)

# Add Lvgl support
add_subdirectory(lvgl)
add_subdirectory(lv_drivers)

target_include_directories(lvgl PUBLIC include/lvgl lv_drivers)
target_include_directories(lv_drivers PUBLIC lvgl)

target_include_directories(node_lvgl_ui PUBLIC ${PROJECT_SOURCE_DIR} src/generated src/custom src/generated/guider_customer_fonts src/generated/guider_fonts src/generated/images)

# 递归添加子目录${PROJECT_SOURCE_DIR}/generated/*
FILE(GLOB_RECURSE SOURCES src/custom/*.c src/generated/*.c)
target_sources(node_lvgl_ui PUBLIC ${SOURCES})
target_link_libraries(node_lvgl_ui lvgl lv_drivers)

# Add node_can_agent target
add_executable(node_can_agent src/node_can_agent.cpp)
ament_target_dependencies(node_can_agent rclcpp std_msgs info)

# Add libsockcancpp support
add_subdirectory(libsockcancpp)
target_include_directories(node_can_agent PUBLIC libsockcancpp/include libsockcancpp/include/exceptions)
target_link_libraries(node_can_agent libsocketcancpp)

install(TARGETS
    node_lvgl_ui
    node_can_agent
    DESTINATION lib/${PROJECT_NAME})

ament_package()