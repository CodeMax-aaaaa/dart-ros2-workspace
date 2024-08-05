cmake_minimum_required(VERSION 3.5)
project(guider_generated)

# 链接liblvgl
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lvgl/lib/)
link_libraries(lvgl)

# 编译gui guider的generate为共享库
FILE(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.c)
# add_library(guider_generated SHARED ${SOURCES})
add_library(guider_generated STATIC ${SOURCES})

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../../lvgl/include/
    ${CMAKE_CURRENT_LIST_DIR}/../../lvgl/include/lvgl/
    ${CMAKE_CURRENT_LIST_DIR}/../custom
    ${CMAKE_CURRENT_LIST_DIR}
)

# install
install(TARGETS guider_generated
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)