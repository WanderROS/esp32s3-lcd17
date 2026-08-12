/**
 * @file main_pc.cpp
 * PC 模拟器入口
 *
 * 功能对应 src/main.cpp，但剥离了所有 ESP32 硬件依赖：
 *   - LCD (CO5300/QSPI)  → SDL2 窗口
 *   - 触摸 (CST9217)      → SDL2 鼠标事件
 *   - RTC (PCF85063)      → 系统时间 (time.h)
 *   - PMU (AXP2101)       → 键盘快捷键模拟
 *   - I2S 音频 (ES8311)   → SDL2_mixer MP3 播放
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
#include <vector>
#include <string>
#include <algorithm>

/* 目录遍历（POSIX） */
#include <dirent.h>
#include <sys/stat.h>

/* SDL2 */
#include <SDL2/SDL.h>

/* SDL2_mixer（条件编译：找不到时音频功能禁用） */
#ifdef HAVE_SDL2_MIXER
#  include <SDL2/SDL_mixer.h>
#endif

/* LVGL（PC 端使用 sim/lv_conf_pc.h） */
#include "lvgl.h"

/* 项目 UI 资源（与嵌入式端共享） */
#include "lvgl_sd_resource/lvgl_sd_resource.h"

/* ======================================================
 * 编译期检查
 * ====================================================== */
#ifndef PC_SIMULATOR
#  error "main_pc.cpp must be compiled with PC_SIMULATOR defined"
#endif

/* ======================================================
 * 全局参数
 * ====================================================== */
#define SCREEN_WIDTH    466
#define SCREEN_HEIGHT   466
#define DISP_SCALE      1
#define TICK_PERIOD_MS  2
/*
 * ASSETS_PATH：LVGL FS 驱动 'A' 映射到的本地根目录（相对可执行文件）。
 * 设为 "" → 映射到当前工作目录，"A:assets/fonts/xxx" → "./assets/fonts/xxx"。
 */
#define ASSETS_PATH     ""
/* 音乐目录（相对可执行文件），对应嵌入式端 SD 卡根目录 */
#define MUSIC_DIR       "assets/music"

static lv_display_t *g_disp   = nullptr;
static lv_indev_t   *g_indev  = nullptr;

/* ======================================================
 * SDL2 上下文
 * ====================================================== */
static SDL_Window   *g_window   = nullptr;
static SDL_Renderer *g_renderer = nullptr;
static SDL_Texture  *g_texture  = nullptr;
static uint32_t     *g_px_buf   = nullptr;

/* ======================================================
 * 音频上下文
 * ====================================================== */
#ifdef HAVE_SDL2_MIXER

static std::vector<std::string> g_mp3_files;   /* 扫描到的 MP3 列表 */
static int                      g_mp3_index = 0;
static Mix_Music               *g_music     = nullptr;
static bool                     g_audio_ok  = false;

/* 前向声明 */
static void SDLCALL audio_music_finished_cb(void);

/* 扫描目录，收集所有 .mp3 文件（不区分大小写） */
static void audio_scan_music_dir(const char *dir_path)
{
    DIR *d = opendir(dir_path);
    if (!d) {
        printf("[Audio] Music dir not found: %s  (put MP3s there to enable playback)\n",
               dir_path);
        return;
    }
    struct dirent *ent;
    while ((ent = readdir(d)) != nullptr) {
        if (ent->d_type != DT_REG && ent->d_type != DT_UNKNOWN) continue;
        std::string name(ent->d_name);
        /* 不区分大小写匹配 .mp3 */
        if (name.size() > 4) {
            std::string ext = name.substr(name.size() - 4);
            for (auto &c : ext) c = (char)tolower((unsigned char)c);
            if (ext == ".mp3") {
                g_mp3_files.push_back(std::string(dir_path) + "/" + name);
            }
        }
    }
    closedir(d);

    /* 按文件名排序，与嵌入式端目录读取顺序保持一致 */
    std::sort(g_mp3_files.begin(), g_mp3_files.end());

    printf("[Audio] Found %zu MP3 file(s) in %s\n", g_mp3_files.size(), dir_path);
    for (const auto &f : g_mp3_files) {
        printf("        %s\n", f.c_str());
    }
}

/* 播放指定索引的 MP3 */
static void audio_play_index(int idx)
{
    if (g_mp3_files.empty()) return;

    /* 先注销回调，防止 Mix_HaltMusic() 触发 finished 回调造成递归切歌 */
    Mix_HookMusicFinished(nullptr);

    if (g_music) {
        Mix_HaltMusic();
        Mix_FreeMusic(g_music);
        g_music = nullptr;
    }

    const char *path = g_mp3_files[idx].c_str();
    g_music = Mix_LoadMUS(path);
    if (!g_music) {
        fprintf(stderr, "[Audio] Mix_LoadMUS failed: %s  (%s)\n",
                path, Mix_GetError());
        Mix_HookMusicFinished(audio_music_finished_cb);
        return;
    }

    if (Mix_PlayMusic(g_music, 1) != 0) {
        fprintf(stderr, "[Audio] Mix_PlayMusic failed: %s\n", Mix_GetError());
        Mix_HookMusicFinished(audio_music_finished_cb);
        return;
    }
    printf("[Audio] Playing: %s\n", path);

    /* 重新注册回调，等待本曲播放结束后自动切下一首 */
    Mix_HookMusicFinished(audio_music_finished_cb);

    /* 更新 LVGL Observer（song_playing = 1 表示播放中） */
    lv_subject_set_int(&song_playing, 1);
}

/* 切歌回调（在 SDL_mixer 曲目结束时由 audio 线程调用） */
static void SDLCALL audio_music_finished_cb(void)
{
    if (g_mp3_files.empty()) return;
    g_mp3_index = (g_mp3_index + 1) % (int)g_mp3_files.size();
    /* Mix_HookMusicFinished 回调不能直接调用 Mix_PlayMusic，
     * 用 SDL 用户事件在主线程延迟处理 */
    SDL_Event ev;
    SDL_zero(ev);
    ev.type = SDL_USEREVENT;
    ev.user.code = 1;   /* code=1: 播放下一首 */
    SDL_PushEvent(&ev);
}

/* 初始化 SDL2_mixer 并开始播放 */
static void audio_init(void)
{
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) != 0) {
        fprintf(stderr, "[Audio] Mix_OpenAudio failed: %s\n", Mix_GetError());
        return;
    }
    Mix_VolumeMusic(90);    /* 对应嵌入式端 setVolume(6)，约 70% */

    audio_scan_music_dir(MUSIC_DIR);

    if (!g_mp3_files.empty()) {
        Mix_HookMusicFinished(audio_music_finished_cb);
        audio_play_index(g_mp3_index);
    }
    g_audio_ok = true;
}

/* 清理 */
static void audio_deinit(void)
{
    if (g_music) {
        Mix_HaltMusic();
        Mix_FreeMusic(g_music);
        g_music = nullptr;
    }
    Mix_CloseAudio();
}

#endif /* HAVE_SDL2_MIXER */

/* ======================================================
 * LVGL 显示刷新回调（SDL2 后端）
 * ====================================================== */
static void sdl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);

    uint32_t *src = (uint32_t *)px_map;
    for (int32_t y = 0; y < h; y++) {
        uint32_t *dst_row = g_px_buf + (area->y1 + y) * SCREEN_WIDTH + area->x1;
        memcpy(dst_row, src + y * w, w * sizeof(uint32_t));
    }

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
 * LVGL 鼠标输入读取回调
 * ====================================================== */
static void sdl_indev_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    int mx, my;
    uint32_t buttons = SDL_GetMouseState(&mx, &my);
    data->point.x = (lv_coord_t)(mx / DISP_SCALE);
    data->point.y = (lv_coord_t)(my / DISP_SCALE);
    data->state   = (buttons & SDL_BUTTON_LMASK)
                    ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

/* ======================================================
 * LVGL 文件系统驱动（A: → 当前工作目录）
 * ====================================================== */
static void *pc_fs_open_cb(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode)
{
    const char *flags = (mode == LV_FS_MODE_WR) ? "wb" : "rb";
    char full_path[512];
    snprintf(full_path, sizeof(full_path), "%s%s", ASSETS_PATH, path);
    FILE *f = fopen(full_path, flags);
    if (!f) fprintf(stderr, "[FS] open failed: %s\n", full_path);
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
    drv.letter   = 'A';
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
    uint32_t flags = SDL_INIT_VIDEO | SDL_INIT_EVENTS;
#ifdef HAVE_SDL2_MIXER
    flags |= SDL_INIT_AUDIO;
#endif
    if (SDL_Init(flags) != 0) {
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

    SDL_RenderSetLogicalSize(g_renderer,
                             SCREEN_WIDTH * DISP_SCALE,
                             SCREEN_HEIGHT * DISP_SCALE);
    return true;
}

/* ======================================================
 * 初始化 LVGL
 * ====================================================== */
static void lvgl_init(void)
{
    lv_init();

    lv_color_format_t cf = LV_COLOR_FORMAT_NATIVE;
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

    g_indev = lv_indev_create();
    lv_indev_set_type(g_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(g_indev, sdl_indev_read_cb);
}

/* ======================================================
 * 创建时钟屏幕
 * ====================================================== */
static void create_clock_screen(void)
{
    lv_obj_t *clock_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(clock_scr, lv_color_black(), 0);

    lv_obj_t *clock_label = lv_label_create(clock_scr);
    lv_obj_set_style_text_color(clock_label, lv_color_white(), 0);
    if (geist_light_60) lv_obj_set_style_text_font(clock_label, geist_light_60, 0);
    lv_obj_align(clock_label, LV_ALIGN_CENTER, 0, -20);
    lv_label_set_text(clock_label, "00:00:00");
    lv_obj_set_name(clock_label, "clock_label");

    lv_obj_t *date_label = lv_label_create(clock_scr);
    lv_obj_set_style_text_color(date_label, lv_color_hex(0xAAAAAA), 0);
    if (geist_semibold_20) lv_obj_set_style_text_font(date_label, geist_semibold_20, 0);
    lv_obj_align(date_label, LV_ALIGN_CENTER, 0, 40);
    lv_label_set_text(date_label, "2026-01-01");
    lv_obj_set_name(date_label, "date_label");

    lv_screen_load(clock_scr);
}

/* ======================================================
 * 每秒更新时钟
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
 * SDL2 事件处理
 * ====================================================== */
static uint8_t g_current_rotation = 0;

static bool handle_sdl_events(void)
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        switch (e.type) {
            case SDL_QUIT:
                return false;

            /* SDL_USEREVENT: 曲目结束，播放下一首 */
            case SDL_USEREVENT:
#ifdef HAVE_SDL2_MIXER
                if (e.user.code == 1 && g_audio_ok) {
                    audio_play_index(g_mp3_index);
                }
#endif
                break;

            case SDL_KEYDOWN:
                switch (e.key.keysym.sym) {
                    case SDLK_ESCAPE:
                    case SDLK_q:
                        return false;

                    /* R：切换屏幕旋转（模拟电源短按） */
                    case SDLK_r: {
                        g_current_rotation = (g_current_rotation + 1) % 4;
                        lv_display_rotation_t rotations[] = {
                            LV_DISPLAY_ROTATION_0,  LV_DISPLAY_ROTATION_90,
                            LV_DISPLAY_ROTATION_180, LV_DISPLAY_ROTATION_270
                        };
                        lv_display_set_rotation(g_disp, rotations[g_current_rotation]);
                        printf("[SIM] Rotation: %d°\n", g_current_rotation * 90);
                        break;
                    }

                    /* T：切换主题 */
                    case SDLK_t: {
                        int32_t cur = lv_subject_get_int(&dark_theme);
                        lv_subject_set_int(&dark_theme, cur ? 0 : 1);
                        printf("[SIM] Theme: %s\n", cur ? "light" : "dark");
                        break;
                    }

#ifdef HAVE_SDL2_MIXER
                    /* 空格：暂停 / 继续 */
                    case SDLK_SPACE: {
                        if (!g_audio_ok || g_mp3_files.empty()) break;
                        if (Mix_PausedMusic()) {
                            Mix_ResumeMusic();
                            lv_subject_set_int(&song_playing, 1);
                            printf("[Audio] Resumed\n");
                        } else {
                            Mix_PauseMusic();
                            lv_subject_set_int(&song_playing, 0);
                            printf("[Audio] Paused\n");
                        }
                        break;
                    }

                    /* N：下一首 */
                    case SDLK_n: {
                        if (!g_audio_ok || g_mp3_files.empty()) break;
                        g_mp3_index = (g_mp3_index + 1) % (int)g_mp3_files.size();
                        audio_play_index(g_mp3_index);
                        break;
                    }

                    /* P：上一首 */
                    case SDLK_p: {
                        if (!g_audio_ok || g_mp3_files.empty()) break;
                        g_mp3_index = ((g_mp3_index - 1) +
                                       (int)g_mp3_files.size()) % (int)g_mp3_files.size();
                        audio_play_index(g_mp3_index);
                        break;
                    }

                    /* 上箭头：音量 +10% */
                    case SDLK_UP: {
                        if (!g_audio_ok) break;
                        int vol = Mix_VolumeMusic(-1);
                        vol = std::min(vol + 13, MIX_MAX_VOLUME);
                        Mix_VolumeMusic(vol);
                        int32_t pct = vol * 100 / MIX_MAX_VOLUME;
                        lv_subject_set_int(&speaker_vol, pct);
                        printf("[Audio] Volume: %d%%\n", pct);
                        break;
                    }

                    /* 下箭头：音量 -10% */
                    case SDLK_DOWN: {
                        if (!g_audio_ok) break;
                        int vol = Mix_VolumeMusic(-1);
                        vol = std::max(vol - 13, 0);
                        Mix_VolumeMusic(vol);
                        int32_t pct = vol * 100 / MIX_MAX_VOLUME;
                        lv_subject_set_int(&speaker_vol, pct);
                        printf("[Audio] Volume: %d%%\n", pct);
                        break;
                    }
#endif /* HAVE_SDL2_MIXER */

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
 * LVGL Tick 驱动
 * ====================================================== */
static uint32_t g_last_tick_ms = 0;

static void lvgl_tick_update(void)
{
    uint32_t now     = SDL_GetTicks();
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
    printf("  Screen : %dx%d  Scale: %d\n", SCREEN_WIDTH, SCREEN_HEIGHT, DISP_SCALE);
    printf("  Assets : ./%s\n", ASSETS_PATH[0] ? ASSETS_PATH : "(cwd)");
    printf("  Music  : ./%s\n", MUSIC_DIR);
#ifdef HAVE_SDL2_MIXER
    printf("  Audio  : SDL2_mixer enabled\n");
#else
    printf("  Audio  : disabled (SDL2_mixer not found)\n");
#endif
    printf("  Keys   : R=rotate  T=theme  Space=pause  N=next  P=prev\n");
    printf("           Up/Down=volume  Q/ESC=quit\n\n");

    /* 1. SDL2 */
    if (!sdl_init()) return 1;

    /* 2. LVGL */
    lvgl_init();

    /* 3. 文件系统 */
    lv_fs_pc_init();

    /* 4. UI 资源（字体、图片） */
    lvgl_sd_resource_init("A:assets/");

    /* 5. 时钟屏幕 */
    create_clock_screen();

    /* 6. 音频（在 UI 初始化后，确保 Subject 已就绪） */
#ifdef HAVE_SDL2_MIXER
    audio_init();
#endif

    /* 7. 主循环 */
    g_last_tick_ms  = SDL_GetTicks();
    g_last_clock_ms = SDL_GetTicks();

    while (true) {
        if (!handle_sdl_events()) break;

        lvgl_tick_update();
        uint32_t delay_ms = lv_timer_handler();
        update_clock();

        if (delay_ms > 0 && delay_ms < 5) delay_ms = 5;
        SDL_Delay(delay_ms);
    }

    /* 8. 清理 */
    printf("[SIM] Exiting...\n");
#ifdef HAVE_SDL2_MIXER
    audio_deinit();
#endif
    free(g_px_buf);
    SDL_DestroyTexture(g_texture);
    SDL_DestroyRenderer(g_renderer);
    SDL_DestroyWindow(g_window);
    SDL_Quit();

    return 0;
}
