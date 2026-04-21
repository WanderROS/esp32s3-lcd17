#pragma once
/**
 * map_screen.h — 地图界面
 *
 * 布局：
 *   - 3×3 瓦片拼接（768×768），居中裁剪到 466×466 屏幕
 *   - 中心红点标记当前位置
 *   - 底部信息栏：经纬度 + 缩放级别
 *   - 右上角 +/- 缩放按钮
 *
 * JPEG 解码：使用 TJpgDec（ESP32 Arduino 内置）
 * 解码输出 RGB565 → 直接写入 LVGL Canvas 缓冲区
 */

#include <Arduino.h>
#include <lvgl.h>
#include <TJpg_Decoder.h>
#include "tile_map.h"
#include "pin_config.h"

// ── Canvas 尺寸（3×3 瓦片）────────────────────────────────
#define MAP_CANVAS_W  (TILE_SIZE * 3)   // 768
#define MAP_CANVAS_H  (TILE_SIZE * 3)   // 768

// ── JPEG 解码回调上下文 ───────────────────────────────────
// TJpgDec 解码时逐行回调，我们把 RGB565 写入 canvas 缓冲区
static lv_color_t *s_canvas_buf = nullptr;
static int s_tile_ox = 0;   // 当前瓦片在 canvas 中的 x 偏移（像素）
static int s_tile_oy = 0;   // 当前瓦片在 canvas 中的 y 偏移（像素）

// TJpgDec 输出回调：每次给一行 MCU 块（通常 8 或 16 行）
static bool jpegOutputCb(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
    if (!s_canvas_buf) return false;
    for (int row = 0; row < h; row++) {
        int cy = s_tile_oy + y + row;
        if (cy < 0 || cy >= MAP_CANVAS_H) continue;
        for (int col = 0; col < w; col++) {
            int cx = s_tile_ox + x + col;
            if (cx < 0 || cx >= MAP_CANVAS_W) continue;
            // RGB565 大端 → LVGL lv_color_t（小端 RGB565）
            uint16_t px = bitmap[row * w + col];
            // TJpgDec 输出已是 RGB565，直接赋值
            s_canvas_buf[cy * MAP_CANVAS_W + cx].full = px;
        }
    }
    return true;
}

// ── MapScreen 类 ──────────────────────────────────────────
// 静态指针：用于后台任务通知主线程刷新（LVGL v8 timer 无 user_data getter）
class MapScreen;
static MapScreen *s_pending_refresh = nullptr;

class MapScreen {
public:
    lv_obj_t *screen   = nullptr;
    lv_obj_t *canvas   = nullptr;
    lv_obj_t *marker   = nullptr;
    lv_obj_t *info_lbl = nullptr;
    lv_obj_t *btn_plus = nullptr;
    lv_obj_t *btn_minus= nullptr;

    TileMap tileMap;

    double _lat  = 39.9042;    // 默认：北京天安门
    double _lon  = 116.4074;
    int    _zoom = MAP_ZOOM_DEFAULT;

    bool _needsUpdate = false;  // 标记需要刷新地图

    // ── 初始化 ────────────────────────────────────────────
    void begin(lv_obj_t *parent) {
        tileMap.begin();

        // 初始化 TJpgDec
        TJpgDec.setJpgScale(1);
        TJpgDec.setSwapBytes(false);  // LVGL RGB565 不需要字节交换
        TJpgDec.setCallback(jpegOutputCb);

        // 分配 Canvas 缓冲区（PSRAM，768×768×2 ≈ 1.1 MB）
        if (!s_canvas_buf) {
            s_canvas_buf = (lv_color_t*)ps_malloc(
                MAP_CANVAS_W * MAP_CANVAS_H * sizeof(lv_color_t));
            if (!s_canvas_buf) {
                Serial.println("[MAP] Canvas 缓冲区分配失败！");
                return;
            }
            memset(s_canvas_buf, 0x1a, MAP_CANVAS_W * MAP_CANVAS_H * sizeof(lv_color_t));
        }

        screen = parent;
        // 开启裁剪，canvas 超出屏幕边界的部分不渲染
        lv_obj_set_style_clip_corner(screen, false, 0);
        lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

        // Canvas 设为完整 3×3 瓦片大小（768×768）
        // 通过负坐标定位，使中心瓦片（256~512区域）对准屏幕中心
        // 偏移量 = -(768 - 466) / 2 = -151
        int offset = -((MAP_CANVAS_W - LCD_WIDTH) / 2);  // -151

        canvas = lv_canvas_create(screen);
        lv_canvas_set_buffer(canvas, s_canvas_buf,
                             MAP_CANVAS_W, MAP_CANVAS_H, LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_size(canvas, MAP_CANVAS_W, MAP_CANVAS_H);
        lv_obj_set_pos(canvas, offset, offset);  // 左上角偏移，中心瓦片居中

        // 位置标记（红圆点，固定在屏幕中心）
        marker = lv_obj_create(screen);
        lv_obj_set_size(marker, 18, 18);
        lv_obj_set_style_radius(marker, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(marker, lv_color_make(220, 50, 50), 0);
        lv_obj_set_style_border_color(marker, lv_color_white(), 0);
        lv_obj_set_style_border_width(marker, 2, 0);
        lv_obj_set_style_shadow_width(marker, 8, 0);
        lv_obj_set_style_shadow_color(marker, lv_color_make(220, 50, 50), 0);
        lv_obj_center(marker);

        // 信息栏（底部）
        info_lbl = lv_label_create(screen);
        lv_obj_set_style_bg_color(info_lbl, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(info_lbl, LV_OPA_70, 0);
        lv_obj_set_style_text_color(info_lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(info_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_pad_hor(info_lbl, 8, 0);
        lv_obj_set_style_pad_ver(info_lbl, 4, 0);
        lv_obj_set_style_radius(info_lbl, 6, 0);
        lv_label_set_text(info_lbl, "Loading map...");
        lv_obj_align(info_lbl, LV_ALIGN_BOTTOM_MID, 0, -8);

        // 缩放按钮
        _createZoomBtn(btn_plus,  "+", LV_ALIGN_TOP_RIGHT, -10, 10,
            [](lv_event_t *e) {
                MapScreen *ms = (MapScreen*)lv_event_get_user_data(e);
                if (ms->_zoom < 18) { ms->_zoom++; ms->_needsUpdate = true; }
            });
        _createZoomBtn(btn_minus, "-", LV_ALIGN_TOP_RIGHT, -10, 60,
            [](lv_event_t *e) {
                MapScreen *ms = (MapScreen*)lv_event_get_user_data(e);
                if (ms->_zoom > 5)  { ms->_zoom--; ms->_needsUpdate = true; }
            });

        Serial.println("[MAP] 地图界面初始化完成");
    }

    // ── 设置坐标（模拟 GPS 或真实 GPS 调用）────────────────
    void setPosition(double lat, double lon) {
        if (fabs(lat - _lat) > 0.00001 || fabs(lon - _lon) > 0.00001) {
            _lat = lat;
            _lon = lon;
            _needsUpdate = true;
        }
    }

    // ── 刷新地图（在后台任务中调用）──────────────────────────
    // 下载瓦片写入 canvas buf（纯内存操作，线程安全）
    // LVGL invalidate 通过 timer 在主线程触发
    bool updateIfNeeded() {
        if (!_needsUpdate) return false;
        _needsUpdate = false;
        _loadTiles();  // HTTP 下载 + 写 canvas buf（不调用 LVGL API）
        // 通过 LVGL timer 在主线程安全地刷新显示
        // LVGL v8 不支持 lv_timer_get_user_data，用静态指针传递
        s_pending_refresh = this;
        lv_timer_t *t = lv_timer_create([](lv_timer_t *tmr) {
            if (s_pending_refresh) {
                s_pending_refresh->_updateInfoLabel();
                if (s_pending_refresh->canvas)
                    lv_obj_invalidate(s_pending_refresh->canvas);
                s_pending_refresh = nullptr;
            }
            lv_timer_del(tmr);
        }, 10, nullptr);
        (void)t;
        return true;
    }

    // ── 强制立即刷新（在 LVGL 主线程调用）──────────────────
    void forceUpdate() {
        _needsUpdate = true;
        // 首次加载在调用线程同步执行（setup 阶段，WiFi 已连接）
        _loadTiles();
        _updateInfoLabel();
        if (canvas) lv_obj_invalidate(canvas);
        _needsUpdate = false;
    }

private:
    // 下载并绘制 3×3 瓦片到 canvas
    void _loadTiles() {
        TileXY center = latLonToTile(_lat, _lon, _zoom);
        Serial.printf("[MAP] 加载瓦片 center=(%d,%d) zoom=%d\n", center.x, center.y, _zoom);

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int tx = center.x + dx;
                int ty = center.y + dy;
                int ox = (dx + 1) * TILE_SIZE;
                int oy = (dy + 1) * TILE_SIZE;

                size_t len = 0;
                uint8_t *jpg = tileMap.getTile(tx, ty, _zoom, len);
                if (jpg && len > 0) {
                    _drawJpegToCanvas(jpg, len, ox, oy);
                    free(jpg);
                } else {
                    // 填充灰色占位
                    _fillRect(ox, oy, TILE_SIZE, TILE_SIZE, lv_color_hex(0x2a2a3a));
                }
            }
        }
    }

    // 解码 JPEG 并写入 canvas 指定位置
    void _drawJpegToCanvas(uint8_t *data, size_t len, int ox, int oy) {
        s_tile_ox = ox;
        s_tile_oy = oy;
        // TJpgDec 从内存解码，回调写入 s_canvas_buf
        TJpgDec.drawJpg(0, 0, data, len);
    }

    // 填充纯色矩形到 canvas 缓冲区
    void _fillRect(int x, int y, int w, int h, lv_color_t color) {
        for (int row = y; row < y + h && row < MAP_CANVAS_H; row++) {
            for (int col = x; col < x + w && col < MAP_CANVAS_W; col++) {
                s_canvas_buf[row * MAP_CANVAS_W + col] = color;
            }
        }
    }

    void _updateInfoLabel() {
        char buf[64];
        snprintf(buf, sizeof(buf), "%.5f, %.5f  Z:%d", _lat, _lon, _zoom);
        lv_label_set_text(info_lbl, buf);
        lv_obj_align(info_lbl, LV_ALIGN_BOTTOM_MID, 0, -8);
    }

    void _createZoomBtn(lv_obj_t *&btn, const char *txt,
                        lv_align_t align, int ox, int oy,
                        lv_event_cb_t cb) {
        btn = lv_btn_create(screen);
        lv_obj_set_size(btn, 40, 40);
        lv_obj_align(btn, align, ox, oy);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x222244), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x4444aa), LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x6666cc), 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, this);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
        lv_obj_center(lbl);
    }
};
