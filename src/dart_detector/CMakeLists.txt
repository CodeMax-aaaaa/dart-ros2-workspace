cmake_minimum_required(VERSION 3.8)
project(dart_detector)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dart_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 环境变量 DART_DEVICE_TYPE 为 dart_launcher 时：
if(DEFINED ENV{DART_DEVICE_TYPE})
        message(STATUS "DART_DEVICE_TYPE: $ENV{DART_DEVICE_TYPE}")
        if($ENV{DART_DEVICE_TYPE} STREQUAL "dart_launcher")
                message(STATUS "dart_launcher")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDART_LAUNCHER")
        elseif($ENV{DART_DEVICE_TYPE} STREQUAL "guided_dart")
                message(STATUS "guided_dart")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGUIDED_DART")
        endif()
endif()

# ===== 视觉检测 OpenCV相关 开始 =====
set(SOURCES src/node_detector.cpp src/detector/greenlight_detect.cpp)

add_executable(dart_detector_node ${SOURCES})

ament_target_dependencies(
        dart_detector_node
        rclcpp
        std_msgs
        rclcpp_lifecycle
        geometry_msgs
        dart_msgs
        sensor_msgs
        cv_bridge
)

target_include_directories(
        dart_detector_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/
        ${OpenCV_INCLUDE_DIRS})

target_link_libraries(dart_detector_node ${OpenCV_LIBRARIES})

# ===== 视觉检测 OpenCV相关 结束 =====

# ===== 相机节点 根据不同设备类型编译不同的相机驱动 开始 =====
ament_target_dependencies(camera_node rclcpp sensor_msgs
        cv_bridge rclcpp_lifecycle)

target_include_directories(
        camera_node
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
        ${OpenCV_INCLUDE_DIRS})

if($ENV{DART_DEVICE_TYPE} STREQUAL "dart_launcher")
        # 添加宏定义
        add_definitions(-DDART_LAUNCHER)

        # LCCV
        pkg_check_modules(LIBCAMERA REQUIRED libcamera)
        target_include_directories(
                camera_node
                PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/LCCV/include
                ${LIBCAMERA_INCLUDE_DIRS}
        )

        FILE(GLOB SOURCES_HAL_CAMERA src/camera_hal/camera_dh.cpp src/camera_hal/camera_lccv.cpp camera_driver/LCCV/src/*.cpp)
        add_executable(camera_node src/node_camera.cpp camera_driver/DHCameraDriver/DHVideoCapture.cpp ${SOURCES_HAL_CAMERA})

        # structure judge
        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
                message(STATUS "aarch64")
                target_include_directories(
                        camera_node
                        PRIVATE
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/DHCameraDriver/DHCamera_arm64 # here
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/
                        ${OpenCV_INCLUDE_DIRS})

                target_link_directories(
                        camera_node PRIVATE
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/DHCameraDriver/DHCamera_arm64/lib
                )
                set(MVSDK_LIBS
                        log4cpp
                        log4cpp_gcc485_v3_0
                        GCBase_gcc485_v3_0
                        GenApi_gcc485_v3_0
                        Log_gcc485_v3_0
                        MathParser_gcc485_v3_0
                        XmlParser_gcc485_v3_0
                        NodeMapData_gcc485_v3_0
                        ImageConvert
                )
                target_link_libraries(camera_node ${OpenCV_LIBS} MVSDK ${MVSDK_LIBS} pthread ${LIBCAMERA_LINK_LIBRARIES})
        elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
                message(STATUS "x86_64")
                target_include_directories(
                        camera_node
                        PRIVATE
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/DHCameraDriver/DHCamera # here
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/
                        ${OpenCV_INCLUDE_DIRS})

                target_link_directories(
                        camera_node PRIVATE
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/DHCameraDriver/DHCamera/lib
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/DHCameraDriver/DHCamera
                        ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/DHCameraDriver/DHCamera/lib/GenICam/bin/Linux64_x64
                )
                set(MVSDK_LIBS
                        log4cpp
                        GCBase_gcc421_v3_0
                        GenApi_gcc421_v3_0
                        Log_gcc421_v3_0
                        MathParser_gcc421_v3_0
                        XmlParser_gcc421_v3_0
                        NodeMapData_gcc421_v3_0
                )
                target_link_libraries(camera_node ${OpenCV_LIBS} MVSDK ${MVSDK_LIBS} pthread ${LIBCAMERA_LINK_LIBRARIES})
        endif()
endif()
# ===== 相机节点 根据不同设备类型编译不同的相机驱动 结束 =====

# ===== 参数调节节点 开始 =====
# add_executable(parameter_node src/parameter_tuner.cpp)
# target_include_directories(parameter_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_link_libraries(parameter_node ${OpenCV_LIBRARIES})
# target_sources(parameter_node PRIVATE src/detect.cpp)
# ament_target_dependencies(parameter_node)
# ===== 参数调节节点 结束 =====

install(TARGETS dart_detector_node camera_node # parameter_node
        DESTINATION lib/${PROJECT_NAME})

# 根据设备类型决定导出需要的共享库
if($ENV{DART_DEVICE_TYPE} STREQUAL "dart_launcher")
        if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
                install(DIRECTORY
                        camera_driver/DHCameraDriver/DHCamera_arm64/lib/
                        DESTINATION lib
                        FILES_MATCHING
                        PATTERN "*.so"
                )

        elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
                install(DIRECTORY
                        camera_driver/DHCameraDriver/DHCamera/lib/
                        DESTINATION lib
                        FILES_MATCHING
                        PATTERN "*.so"
                )
        endif()
endif()


if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)

        # the following line skips the linter which checks for copyrights comment the
        # line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)

        # the following line skips cpplint (only works in a git repo) comment the line
        # when this package is in a git repo and when a copyright and  license is
        # added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
