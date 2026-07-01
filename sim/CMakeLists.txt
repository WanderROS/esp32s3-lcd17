cmake_minimum_required(VERSION 3.16)
project(esp32s3_lcd17_sim CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD   11)

# ============================================================
# 路径别名
# ============================================================
set(ROOT_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/..)  # 项目根目录
set(SIM_DIR     ${CMAKE_CURRENT_SOURCE_DIR})     # sim/ 目录
set(SRC_DIR     ${ROOT_DIR}/src)
set(LIB_DIR     ${ROOT_DIR}/lib)
set(LVGL_DIR    ${LIB_DIR}/lvgl)

# ============================================================
# 查找 SDL2
# macOS (homebrew): brew install sdl2
# Ubuntu/Debian:    sudo apt install libsdl2-dev
# ============================================================
find_package(SDL2 REQUIRED)
if(NOT SDL2_FOUND)
    message(FATAL_ERROR "SDL2 not found. Install it:\n"
            "  macOS:  brew install sdl2\n"
            "  Ubuntu: sudo apt install libsdl2-dev")
endif()

# ============================================================
# LVGL 配置
# 使用 LV_BUILD_CONF_DIR 方式：sim/ 目录下有 lv_conf.h（转发到 lv_conf_pc.h）
# LVGL 会用 LV_CONF_INCLUDE_SIMPLE 模式从该目录找 lv_conf.h
# ============================================================
set(LV_BUILD_CONF_DIR "${SIM_DIR}" CACHE PATH "" FORCE)
# 关闭 demos/examples 加快编译（如需 lv_demo_widgets 可改为 ON）
set(CONFIG_LV_BUILD_DEMOS    OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(${LVGL_DIR} ${CMAKE_BINARY_DIR}/lvgl)

# ============================================================
# UI 组件库源文件（与 lvgl_sd_resource/CMakeLists.txt 等价）
# file_list_gen.cmake 内部使用 ${CMAKE_CURRENT_LIST_DIR}，
# include() 时该变量指向 .cmake 文件所在目录（lvgl_sd_resource/），
# 所以路径解析是正确的。
# ============================================================
include(${SRC_DIR}/lvgl_sd_resource/file_list_gen.cmake)

add_library(ui_lib STATIC ${PROJECT_SOURCES})
target_include_directories(ui_lib PUBLIC
    ${SRC_DIR}                          # for "lvgl_sd_resource/xxx.h"
    ${SRC_DIR}/lvgl_sd_resource
    ${SIM_DIR}                          # for lv_conf_pc.h (via LV_CONF_INCLUDE_SIMPLE)
)
target_compile_definitions(ui_lib PUBLIC
    LV_CONF_INCLUDE_SIMPLE              # LVGL 从 include path 找 lv_conf.h
    LV_LVGL_H_INCLUDE_SIMPLE            # LVGL 头文件使用 "lvgl.h" 而非 "lvgl/lvgl.h"
    PC_SIMULATOR=1
)
target_link_libraries(ui_lib PUBLIC lvgl)

# ============================================================
# 主可执行文件
# ============================================================
add_executable(sim
    ${SIM_DIR}/main_pc.cpp
)

target_include_directories(sim PRIVATE
    ${SIM_DIR}
    ${SRC_DIR}
    ${SRC_DIR}/lvgl_sd_resource
)

target_compile_definitions(sim PRIVATE
    LV_CONF_INCLUDE_SIMPLE
    LV_LVGL_H_INCLUDE_SIMPLE
    PC_SIMULATOR=1
)

target_link_libraries(sim
    ui_lib
    lvgl
    SDL2::SDL2
    SDL2::SDL2main
)

# ============================================================
# 运行时资源：将 assets/ 链接/复制到构建目录
# 这样可执行文件运行时能在 ./assets/ 找到字体和图片
# ============================================================
set(ASSETS_SRC "${ROOT_DIR}/src/lvgl_sd_resource")

# 构建后将 fonts/ 和 images/ 复制到 ${CMAKE_BINARY_DIR}/assets/
add_custom_command(TARGET sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:sim>/assets/fonts"
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:sim>/assets/images"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ASSETS_SRC}/fonts"
        "$<TARGET_FILE_DIR:sim>/assets/fonts"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ASSETS_SRC}/images"
        "$<TARGET_FILE_DIR:sim>/assets/images"
    COMMENT "Copying assets to build directory"
)

# ============================================================
# 安装（可选）
# ============================================================
install(TARGETS sim DESTINATION bin)

message(STATUS "")
message(STATUS "=== ESP32S3-LCD17 PC Simulator ===")
message(STATUS "  LVGL dir : ${LVGL_DIR}")
message(STATUS "  Config   : ${SIM_DIR}/lv_conf_pc.h")
message(STATUS "  Assets   : ${ASSETS_SRC}")
message(STATUS "")
message(STATUS "  Build:")
message(STATUS "    cmake -B build sim/")
message(STATUS "    cmake --build build -j$(nproc)")
message(STATUS "    ./build/sim")
message(STATUS "")
