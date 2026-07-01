/**
 * @file main_pc.cpp
 * PC 模拟器入口
 *
 * 功能对应 src/main.cpp，但剥离了所有 ESP32 硬件依赖：
 *   - LCD (CO5300/QSPI)  → SDL2 窗口
 *   - 触摸 (CST9217)      → SDL2 鼠标事件
 *   - RTC (PCF85063)      → 系统时间 (time.h)
 *   - PMU (AXP2101)       → 键盘快捷键模拟
 *   - I2S 音频 (ES8311)   → stub（跳过）
 *   - SD 卡 (SDMMC)       → 本地文件系统（assets/ 目录）
 *   - PSRAM               → 标准 malloc
 *
 * 构建方式：见 sim/CMakeLists.txt
 */

#define PC_SIMULATOR 1  /* 全局平台标识宏 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <chrono>
#include <thread>

/* SDL2 */
#include <SDL2/SDL.h>

/* LVGL（PC 端使用 sim/lv_conf_pc.h） */
#include "lvgl.h"

/* 项目 UI 资源（与嵌入式端共享） */
#include "lvgl_sd_resource/lvgl_sd_resource.h"

/* ======================================================
 * 编译期检查：确保不引入任何嵌入式专用头文件
 * ====================================================== */
#ifndef PC_SIMULATOR
#  error "main_pc.cpp must be compiled with PC_SIMULATOR defined"
#endif

/* ======================================================
 * 全局参数
 * ====================================================== */
#define SCREEN_WIDTH    466
#define SCREEN_HEIGHT   466
#define DISP_SCALE      1       /* 高 DPI 屏幕可设为 2 */
#define TICK_PERIOD_MS  2
/*
 * ASSETS_PATH 是 LVGL FS 驱动字母 'A' 映射到的本地根目录（相对于可执行文件）。
 * 设为 "" 表示映射到当前工作目录，这样 "A:assets/fonts/xxx" → "./assets/fonts/xxx"。
 * 与嵌入式端调用 lvgl_sd_resource_init("A:assets/") 保持一致。
 */
#define ASSETS_PATH     ""  /* 映射到当前目录，路径通过 A:assets/ 传给资源系统 */

static lv_display_t *g_disp   = nullptr;
static lv_indev_t   *g_indev  = nullptr;

/* ======================================================
 * SDL2 上下文
 * ====================================================== */
static SDL_Window   *g_window   = nullptr;
static SDL_Renderer *g_renderer = nullptr;
static SDL_Texture  *g_texture  = nullptr;
static uint32_t     *g_px_buf   = nullptr;   /* ARGB8888 pixel buffer */

/* ======================================================
 * LVGL 显示刷新回调（SDL2 后端）
 * ====================================================== */
static void sdl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    /* px_map 是 XRGB8888（32-bit），与 SDL2 ARGB8888 格式一致 */
    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);

    /* 将 LVGL 缓冲区整行复制到 g_px_buf */
    uint32_t *src = (uint32_t *)px_map;
    for (int32_t y = 0; y < h; y++) {
        uint32_t *dst_row = g_px_buf + (area->y1 + y) * SCREEN_WIDTH + area->x1;
        memcpy(dst_row, src + y * w, w * sizeof(uint32_t));
    }

    /* 全屏刷新时（FULL 模式）直接更新纹理并渲染 */
    if (lv_display_flush_is_last(disp)) {
        SDL_UpdateTexture(g_texture, nullptr, g_px_buf,
                          SCREEN_WIDTH * sizeof(uint32_t));
        SDL_RenderClear(g_renderer);
        SDL_RenderCopy(g_renderer, g_texture, nullptr, nullptr);
        SDL_RenderPresent(g_renderer);
    }

    lv_display_flush_ready(disp);
}

/* ======================================================
 * LVGL 触摸/鼠标输入读取回调
 * ====================================================== */
static void sdl_indev_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    int mx, my;
    uint32_t buttons = SDL_GetMouseState(&mx, &my);

    /* 支持 DISP_SCALE 缩放 */
    data->point.x = (lv_coord_t)(mx / DISP_SCALE);
    data->point.y = (lv_coord_t)(my / DISP_SCALE);
    data->state   = (buttons & SDL_BUTTON_LMASK)
                    ? LV_INDEV_STATE_PR
                    : LV_INDEV_STATE_REL;
}

/* ======================================================
 * LVGL 文件系统驱动（映射到本地 assets/ 目录）
 * 与 src/main.cpp 中的驱动字母和回调接口完全相同，
 * 唯一差异：路径前缀从 /sdcard/ 变为 ASSETS_PATH
 * ====================================================== */
static void *pc_fs_open_cb(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode)
{
    const char *flags = (mode == LV_FS_MODE_WR) ? "wb" : "rb";
    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", ASSETS_PATH, path);
    FILE *f = fopen(full_path, flags);
    if (!f) {
        fprintf(stderr, "[FS] open failed: %s\n", full_path);
    }
    return f;
}

static lv_fs_res_t pc_fs_close_cb(lv_fs_drv_t *drv, void *file_p)
{
    fclose((FILE *)file_p);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pc_fs_read_cb(lv_fs_drv_t *drv, void *file_p,
                                  void *buf, uint32_t btr, uint32_t *br)
{
    *br = (uint32_t)fread(buf, 1, btr, (FILE *)file_p);
    return (*br > 0 || btr == 0) ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
}

static lv_fs_res_t pc_fs_seek_cb(lv_fs_drv_t *drv, void *file_p,
                                  uint32_t pos, lv_fs_whence_t whence)
{
    int w;
    switch (whence) {
        case LV_FS_SEEK_SET: w = SEEK_SET; break;
        case LV_FS_SEEK_CUR: w = SEEK_CUR; break;
        case LV_FS_SEEK_END: w = SEEK_END; break;
        default: return LV_FS_RES_INV_PARAM;
    }
    fseek((FILE *)file_p, (long)pos, w);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pc_fs_tell_cb(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p)
{
    *pos_p = (uint32_t)ftell((FILE *)file_p);
    return LV_FS_RES_OK;
}

static void lv_fs_pc_init(void)
{
    static lv_fs_drv_t drv;
    lv_fs_drv_init(&drv);
    drv.letter   = 'A';           /* 与嵌入式端驱动字母保持一致 */
    drv.open_cb  = pc_fs_open_cb;
    drv.close_cb = pc_fs_close_cb;
    drv.read_cb  = pc_fs_read_cb;
    drv.seek_cb  = pc_fs_seek_cb;
    drv.tell_cb  = pc_fs_tell_cb;
    lv_fs_drv_register(&drv);
    printf("[FS] LVGL FS driver 'A' -> ./%s registered.\n", ASSETS_PATH);
}

/* ======================================================
 * 初始化 SDL2
 * ====================================================== */
static bool sdl_init(void)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    g_window = SDL_CreateWindow(
        "ESP32S3-LCD17 Simulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        SCREEN_WIDTH * DISP_SCALE, SCREEN_HEIGHT * DISP_SCALE,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!g_window) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }

    g_renderer = SDL_CreateRenderer(g_window, -1,
                                    SDL_RENDERER_ACCELERATED |
                                    SDL_RENDERER_PRESENTVSYNC);
    if (!g_renderer) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return false;
    }

    /* SDL2 ARGB8888 对应 LVGL XRGB8888（高字节 X 忽略） */
    g_texture = SDL_CreateTexture(g_renderer,
                                  SDL_PIXELFORMAT_ARGB8888,
                                  SDL_TEXTUREACCESS_STREAMING,
                                  SCREEN_WIDTH, SCREEN_HEIGHT);
    if (!g_texture) {
        fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
        return false;
    }

    g_px_buf = (uint32_t *)calloc(SCREEN_WIDTH * SCREEN_HEIGHT, sizeof(uint32_t));
    if (!g_px_buf) {
        fprintf(stderr, "pixel buffer alloc failed\n");
        return false;
    }

    /* 缩放渲染到实际窗口大小 */
    SDL_RenderSetLogicalSize(g_renderer,
                             SCREEN_WIDTH * DISP_SCALE,
                             SCREEN_HEIGHT * DISP_SCALE);
    return true;
}

/* ======================================================
 * 初始化 LVGL + 显示 + 输入设备
 * ====================================================== */
static void lvgl_init(void)
{
    lv_init();

    /* 双缓冲（全屏 FULL 模式）
     * 必须用 lv_draw_buf_width_to_stride() 计算实际 stride，
     * 因为 LVGL 内部可能对行宽进行对齐，使得 stride >= width * bpp。
     * buf_size 必须 >= stride * height，否则 lv_display_set_buffers 会断言失败。
     */
    lv_color_format_t cf = LV_COLOR_FORMAT_NATIVE; /* XRGB8888 for 32-bit depth */
    uint32_t stride   = lv_draw_buf_width_to_stride(SCREEN_WIDTH, cf);
    size_t   buf_size = (size_t)stride * SCREEN_HEIGHT;

    lv_color_t *buf1 = (lv_color_t *)malloc(buf_size);
    lv_color_t *buf2 = (lv_color_t *)malloc(buf_size);
    if (!buf1 || !buf2) {
        fprintf(stderr, "LVGL draw buffer alloc failed\n");
        exit(1);
    }

    g_disp = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_display_set_flush_cb(g_disp, sdl_flush_cb);
    lv_display_set_buffers(g_disp, buf1, buf2, buf_size,
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_rotation(g_disp, LV_DISPLAY_ROTATION_0);

    /* 鼠标输入设备 */
    g_indev = lv_indev_create();
    lv_indev_set_type(g_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(g_indev, sdl_indev_read_cb);
}

/* ======================================================
 * 创建时钟屏幕（与 src/main.cpp setup() 中相同逻辑）
 * ====================================================== */
static void create_clock_screen(void)
{
    lv_obj_t *clock_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(clock_scr, lv_color_black(), 0);

    lv_obj_t *clock_label = lv_label_create(clock_scr);
    lv_obj_set_style_text_color(clock_label, lv_color_white(), 0);
    /* geist_light_60 在 PC 端也通过文件系统加载，若加载失败则回退默认字体 */
    if (geist_light_60) {
        lv_obj_set_style_text_font(clock_label, geist_light_60, 0);
    }
    lv_obj_align(clock_label, LV_ALIGN_CENTER, 0, -20);
    lv_label_set_text(clock_label, "00:00:00");
    lv_obj_set_name(clock_label, "clock_label");

    lv_obj_t *date_label = lv_label_create(clock_scr);
    lv_obj_set_style_text_color(date_label, lv_color_hex(0xAAAAAA), 0);
    if (geist_semibold_20) {
        lv_obj_set_style_text_font(date_label, geist_semibold_20, 0);
    }
    lv_obj_align(date_label, LV_ALIGN_CENTER, 0, 40);
    lv_label_set_text(date_label, "2026-01-01");
    lv_obj_set_name(date_label, "date_label");

    lv_screen_load(clock_scr);
}

/* ======================================================
 * 每秒更新时钟（用系统时间替代 RTC）
 * ====================================================== */
static uint32_t g_last_clock_ms = 0;
static char     g_disp_buf[64];

static void update_clock(void)
{
    uint32_t now_ms = (uint32_t)(SDL_GetTicks());
    if (now_ms - g_last_clock_ms < 1000) return;
    g_last_clock_ms = now_ms;

    time_t t = time(nullptr);
    struct tm *lt = localtime(&t);

    lv_obj_t *scr = lv_screen_active();
    lv_obj_t *cl  = lv_obj_get_child_by_name(scr, "clock_label");
    lv_obj_t *dl  = lv_obj_get_child_by_name(scr, "date_label");

    if (cl) {
        snprintf(g_disp_buf, sizeof(g_disp_buf),
                 "%02d:%02d:%02d", lt->tm_hour, lt->tm_min, lt->tm_sec);
        lv_label_set_text(cl, g_disp_buf);
    }
    if (dl) {
        snprintf(g_disp_buf, sizeof(g_disp_buf),
                 "%04d-%02d-%02d",
                 lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday);
        lv_label_set_text(dl, g_disp_buf);
    }
}

/* ======================================================
 * 处理 SDL2 事件（窗口关闭、键盘模拟 PMU 按键等）
 * 返回 false 表示退出
 * ====================================================== */
static uint8_t g_current_rotation = 0;

static bool handle_sdl_events(void)
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        switch (e.type) {
            case SDL_QUIT:
                return false;

            case SDL_KEYDOWN:
                switch (e.key.keysym.sym) {
                    case SDLK_ESCAPE:
                    case SDLK_q:
                        return false;

                    /* 模拟短按电源键：切换屏幕旋转 */
                    case SDLK_r: {
                        g_current_rotation = (g_current_rotation + 1) % 4;
                        lv_display_rotation_t rotations[] = {
                            LV_DISPLAY_ROTATION_0,
                            LV_DISPLAY_ROTATION_90,
                            LV_DISPLAY_ROTATION_180,
                            LV_DISPLAY_ROTATION_270
                        };
                        lv_display_set_rotation(g_disp, rotations[g_current_rotation]);
                        printf("[SIM] Rotation: %d degrees\n",
                               g_current_rotation * 90);
                        break;
                    }

                    /* 模拟切换深色/浅色主题 */
                    case SDLK_t: {
                        int32_t cur = lv_subject_get_int(&dark_theme);
                        lv_subject_set_int(&dark_theme, cur ? 0 : 1);
                        printf("[SIM] Theme: %s\n", cur ? "light" : "dark");
                        break;
                    }

                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }
    return true;
}

/* ======================================================
 * LVGL Tick 驱动（基于 SDL_GetTicks）
 * ====================================================== */
static uint32_t g_last_tick_ms = 0;

static void lvgl_tick_update(void)
{
    uint32_t now = SDL_GetTicks();
    uint32_t elapsed = now - g_last_tick_ms;
    if (elapsed > 0) {
        lv_tick_inc(elapsed);
        g_last_tick_ms = now;
    }
}

/* ======================================================
 * main
 * ====================================================== */
int main(int argc, char *argv[])
{
    printf("=== ESP32S3-LCD17 PC Simulator ===\n");
    printf("  Screen: %dx%d  Scale: %d\n",
           SCREEN_WIDTH, SCREEN_HEIGHT, DISP_SCALE);
    printf("  Assets: ./%s\n", ASSETS_PATH);
    printf("  Keys:  R=rotate  T=theme  Q/ESC=quit\n\n");

    /* 1. SDL2 初始化 */
    if (!sdl_init()) {
        return 1;
    }

    /* 2. LVGL 初始化（含显示和输入设备） */
    lvgl_init();

    /* 3. 注册文件系统驱动（映射 A: → ./assets/） */
    lv_fs_pc_init();

    /* 4. 加载 SD 卡资源（字体、图片，路径与嵌入式端完全一致） */
    /*    "A:assets/" 对应本地 ./assets/assets/，
     *    或者直接传 "A:" 让 lvgl_sd_resource_init 拼接 "assets/" 子目录。
     *    当前嵌入式端调用是 lvgl_sd_resource_init("A:assets/")，
     *    字体路径会变成 "A:assets/fonts/geist_light_60"，
     *    即本地路径 ./assets/assets/fonts/geist_light_60。
     *    建议 assets/ 目录直接放在可执行文件旁，并在其下建 fonts/ images/。
     *    若要保持与嵌入式端完全一致的路径格式，可调整 ASSETS_PATH = "" 使
     *    驱动直接映射到当前目录，字体放 ./assets/fonts/ 即可。
     */
    lvgl_sd_resource_init("A:assets/");

    /* 5. 创建时钟屏幕 */
    create_clock_screen();

    /* 6. 主循环 */
    g_last_tick_ms = SDL_GetTicks();
    g_last_clock_ms = SDL_GetTicks();

    while (true) {
        /* 处理 SDL2 事件 */
        if (!handle_sdl_events()) {
            break;
        }

        /* 更新 LVGL Tick */
        lvgl_tick_update();

        /* 驱动 LVGL 渲染 */
        uint32_t delay_ms = lv_timer_handler();

        /* 更新时钟显示 */
        update_clock();

        /* 限制 CPU 占用（最大约 200fps） */
        if (delay_ms > 0 && delay_ms < 5) delay_ms = 5;
        SDL_Delay(delay_ms);
    }

    /* 7. 清理 */
    printf("[SIM] Exiting...\n");
    free(g_px_buf);
    SDL_DestroyTexture(g_texture);
    SDL_DestroyRenderer(g_renderer);
    SDL_DestroyWindow(g_window);
    SDL_Quit();

    return 0;
}
