#pragma once
/**
 * map_screen.h — 地图界面
 *
 * 布局：
 *   - 3×3 瓦片拼接（768×768），居中裁剪到 466×466 屏幕
 *   - 当前 GPS 点始终固定在屏幕中心，地图相对运动
 *   - 运动轨迹叠加在 canvas 上（蓝色折线 + 历史点）
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
#include <pngle.h>
#include "tile_map.h"
#include "pin_config.h"
#include "SensorQMI8658.hpp"

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

// ── PNG 解码上下文（pngle 回调）────────────────────────────
static void pngDrawCb(pngle_t *pngle, uint32_t x, uint32_t y,
                      uint32_t w, uint32_t h,
                      const uint8_t rgba[4]) {
    if (!s_canvas_buf) return;
    int cx = s_tile_ox + (int)x;
    int cy = s_tile_oy + (int)y;
    if (cx < 0 || cx >= MAP_CANVAS_W || cy < 0 || cy >= MAP_CANVAS_H) return;
    // RGB888 → RGB565
    uint16_t r = rgba[0] >> 3;
    uint16_t g = rgba[1] >> 2;
    uint16_t b = rgba[2] >> 3;
    s_canvas_buf[cy * MAP_CANVAS_W + cx].full = (r << 11) | (g << 5) | b;
}

// ── 轨迹参数 ──────────────────────────────────────────────
#define TRACK_MAX_POINTS  512   // 最多保存 512 个轨迹点

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
    int  _canvas_x = 0;         // canvas 在屏幕上的位置（每次刷新后更新）
    int  _canvas_y = 0;

    // ── IMU / 航向角 ──────────────────────────────────────
    SensorQMI8658 _imu;
    bool   _imuReady    = false;
    float  _heading     = 0.0f;   // 当前航向角（度，顺时针为正，北=0）
    uint32_t _lastImuMs = 0;
    lv_color_t *_arrow_buf = nullptr;  // 箭头 canvas buffer

    // ── 轨迹数据 ──────────────────────────────────────────
    struct TrackPoint { double lat, lon; };
    TrackPoint _track[TRACK_MAX_POINTS];
    int _trackCount = 0;

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
        // canvas 位置在每次刷新时动态调整，使 GPS 点精确落在屏幕中心
        // 初始先用固定偏移，forceUpdate() 会立即修正
        canvas = lv_canvas_create(screen);
        lv_canvas_set_buffer(canvas, s_canvas_buf,
                             MAP_CANVAS_W, MAP_CANVAS_H, LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_size(canvas, MAP_CANVAS_W, MAP_CANVAS_H);
        lv_obj_set_pos(canvas, 0, 0);  // 初始位置，forceUpdate 会修正

        // 位置标记：箭头直接画在地图 canvas buf 上，无背景遮挡
        // 见 _drawArrowOnMap()，在每次 _loadTiles 时调用
        marker = nullptr;
        _arrow_buf = nullptr;
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

        // ── IMU 初始化（QMI8658，共用 Wire） ──────────────
        if (_imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
            _imu.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS,
                                 SensorQMI8658::GYR_ODR_112_1Hz,
                                 SensorQMI8658::LPF_MODE_2);
            _imu.enableGyroscope();
            _imuReady = true;
            _lastImuMs = millis();
            Serial.println("[MAP] QMI8658 初始化成功");
        } else {
            Serial.println("[MAP] QMI8658 初始化失败，地图旋转不可用");
        }
    }

    // ── 设置坐标（模拟 GPS 或真实 GPS 调用）────────────────
    void setPosition(double lat, double lon) {
        if (fabs(lat - _lat) > 0.00001 || fabs(lon - _lon) > 0.00001) {
            _lat = lat;
            _lon = lon;
            _needsUpdate = true;
        }
    }

    // ── 添加轨迹点（每次 GPS 更新时调用）──────────────────
    void addTrackPoint(double lat, double lon) {
        if (_trackCount < TRACK_MAX_POINTS) {
            _track[_trackCount++] = {lat, lon};
        } else {
            // 环形缓冲：丢弃最旧的点
            memmove(_track, _track + 1, (TRACK_MAX_POINTS - 1) * sizeof(TrackPoint));
            _track[TRACK_MAX_POINTS - 1] = {lat, lon};
        }
        setPosition(lat, lon);
    }

    // ── 清除轨迹 ──────────────────────────────────────────
    void clearTrack() {
        _trackCount = 0;
    }

    // ── 更新 IMU 航向角（在 loop 中频繁调用）──────────────
    void updateIMU() {
        if (!_imuReady) return;
        uint32_t now = millis();
        float dt = (now - _lastImuMs) / 1000.0f;
        _lastImuMs = now;
        if (dt <= 0 || dt > 0.5f) return;

        float gx, gy, gz;
        if (_imu.getGyroscope(gx, gy, gz)) {
            float prev = _heading;
            _heading += gz * dt;
            while (_heading >= 360.0f) _heading -= 360.0f;
            while (_heading <    0.0f) _heading += 360.0f;
            // 角度变化超过 0.5° 才触发重绘
            if (fabsf(_heading - prev) > 0.5f) {
                _needsUpdate = true;
            }
        }
    }

    // ── 获取当前航向角（度）──────────────────────────────
    float getHeading() const { return _heading; }

    // ── 直接设置航向角（由 GPS 轨迹方位角驱动）──────────
    void setHeading(float deg) {
        while (deg >= 360.0f) deg -= 360.0f;
        while (deg <    0.0f) deg += 360.0f;
        if (fabsf(deg - _heading) > 0.5f) {
            _heading = deg;
            if (marker) {
                _drawArrowOnMap();  // 下次 _loadTiles 时会重画，这里只更新角度
            }
        }
    }

    // ── 重置航向角为 0 ────────────────────────────────────
    void resetHeading() { _heading = 0.0f; }

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
    // ── GPS 点的全局像素坐标（先转 GCJ-02，再算 Web Mercator）
    void _latLonToGlobalPx(double lat, double lon, double &gx, double &gy) {
        double glat, glon;
        wgs84ToGcj02(lat, lon, glat, glon);
        double n = pow(2.0, _zoom);
        gx = ((glon + 180.0) / 360.0) * n * TILE_SIZE;
        double lat_r = glat * M_PI / 180.0;
        gy = (1.0 - log(tan(lat_r) + 1.0 / cos(lat_r)) / M_PI) / 2.0 * n * TILE_SIZE;
    }

    // 下载并绘制 3×3 瓦片到 canvas，然后动态定位 canvas 并叠加轨迹
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
                    _fillRect(ox, oy, TILE_SIZE, TILE_SIZE, lv_color_hex(0x2a2a3a));
                }
            }
        }

        // ── 计算 canvas 在屏幕上的位置，使 GPS 点精确落在屏幕中心 ──
        // 当前 GPS 点的全局像素坐标
        double gx, gy;
        _latLonToGlobalPx(_lat, _lon, gx, gy);
        // 中心瓦片左上角的全局像素坐标
        double tile_origin_gx = (double)center.x * TILE_SIZE;
        double tile_origin_gy = (double)center.y * TILE_SIZE;
        // GPS 点在 canvas 内的像素坐标（canvas 左上角 = 中心瓦片左上角 - 1个瓦片）
        double gps_in_canvas_x = (gx - tile_origin_gx) + TILE_SIZE;
        double gps_in_canvas_y = (gy - tile_origin_gy) + TILE_SIZE;
        // canvas 左上角在屏幕上的坐标，使 GPS 点落在屏幕中心
        _canvas_x = (int)(LCD_WIDTH  / 2 - gps_in_canvas_x);
        _canvas_y = (int)(LCD_HEIGHT / 2 - gps_in_canvas_y);

        // 叠加轨迹
        _drawTrack();

        // 在 GPS 锚点画朝向箭头（直接写入 canvas buf，无背景遮挡）
        _drawArrowOnMap();
    }

    // ── 将经纬度转换为 canvas 像素坐标 ──────────────────────
    // 以当前 GPS 点为参考：GPS 点在 canvas 内坐标已知，其他点用全局像素差偏移
    bool _latLonToCanvasXY(double lat, double lon, int &cx, int &cy) {
        TileXY center = latLonToTile(_lat, _lon, _zoom);
        double tile_origin_gx = (double)center.x * TILE_SIZE;
        double tile_origin_gy = (double)center.y * TILE_SIZE;

        double gx_cur, gy_cur;
        _latLonToGlobalPx(_lat, _lon, gx_cur, gy_cur);
        double gps_in_canvas_x = (gx_cur - tile_origin_gx) + TILE_SIZE;
        double gps_in_canvas_y = (gy_cur - tile_origin_gy) + TILE_SIZE;

        double gx_pt, gy_pt;
        _latLonToGlobalPx(lat, lon, gx_pt, gy_pt);

        cx = (int)(gps_in_canvas_x + (gx_pt - gx_cur));
        cy = (int)(gps_in_canvas_y + (gy_pt - gy_cur));

        return (cx >= -TILE_SIZE && cx < MAP_CANVAS_W + TILE_SIZE &&
                cy >= -TILE_SIZE && cy < MAP_CANVAS_H + TILE_SIZE);
    }

    // ── 在地图 canvas buf 上画朝向箭头（无背景，直接覆盖地图像素）──
    // GPS 锚点为箭头中心，angle_deg 顺时针，0° 尖端朝上
    void _drawArrowOnMap() {
        // GPS 锚点在 canvas 内的坐标
        int ax = (int)(LCD_WIDTH  / 2 - _canvas_x);
        int ay = (int)(LCD_HEIGHT / 2 - _canvas_y);

        float rad = _heading * M_PI / 180.0f;
        float cos_a = cosf(rad);
        float sin_a = sinf(rad);

        // 导航箭头顶点（本地坐标，0° 时尖端朝上）
        // 形状：尖头 + 两侧翼 + 尾部凹口
        const int N = 5;
        float local[N][2] = {
            {  0,  -20 },   // 0: 尖端（朝向方向）
            { -12,  12 },   // 1: 左翼尾
            {  0,    4 },   // 2: 尾部凹口
            {  12,  12 },   // 3: 右翼尾
            {  0,  -20 },   // 4: 闭合
        };

        // 旋转到 canvas 坐标
        int pts[N][2];
        for (int i = 0; i < N; i++) {
            float lx = local[i][0], ly = local[i][1];
            pts[i][0] = ax + (int)(cos_a * lx - sin_a * ly + 0.5f);
            pts[i][1] = ay + (int)(sin_a * lx + cos_a * ly + 0.5f);
        }

        // 扫描线填充（蓝色主体）
        lv_color_t fill = lv_color_make(30, 160, 255);
        int y_min = MAP_CANVAS_H, y_max = 0;
        for (int i = 0; i < N - 1; i++) {
            if (pts[i][1] < y_min) y_min = pts[i][1];
            if (pts[i][1] > y_max) y_max = pts[i][1];
        }
        y_min = (y_min < 0) ? 0 : y_min;
        y_max = (y_max >= MAP_CANVAS_H) ? MAP_CANVAS_H - 1 : y_max;

        for (int y = y_min; y <= y_max; y++) {
            int x_left = MAP_CANVAS_W, x_right = -1;
            for (int i = 0; i < N - 1; i++) {
                int x0 = pts[i][0],   y0 = pts[i][1];
                int x1 = pts[i+1][0], y1 = pts[i+1][1];
                if ((y0 <= y && y < y1) || (y1 <= y && y < y0)) {
                    int xi = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
                    if (xi < x_left)  x_left  = xi;
                    if (xi > x_right) x_right = xi;
                }
            }
            for (int x = x_left; x <= x_right; x++) {
                if (x >= 0 && x < MAP_CANVAS_W)
                    s_canvas_buf[y * MAP_CANVAS_W + x] = fill;
            }
        }

        // 白色描边
        lv_color_t edge = lv_color_make(255, 255, 255);
        for (int i = 0; i < N - 1; i++) {
            _drawLine(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1], edge, 2);
        }

        // 尖端白色高亮点（强调朝向）
        int tx = pts[0][0], ty = pts[0][1];
        for (int dy = -2; dy <= 2; dy++)
            for (int dx = -2; dx <= 2; dx++)
                if (dx*dx + dy*dy <= 5) {
                    int px = tx+dx, py = ty+dy;
                    if (px >= 0 && px < MAP_CANVAS_W && py >= 0 && py < MAP_CANVAS_H)
                        s_canvas_buf[py * MAP_CANVAS_W + px] = lv_color_make(255, 255, 255);
                }
    }
    void _drawLine(int x0, int y0, int x1, int y1, lv_color_t color, int thickness = 3) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
            // 画粗线：以 (x0,y0) 为中心画 thickness×thickness 方块
            int half = thickness / 2;
            for (int ry = -half; ry <= half; ry++) {
                for (int rx = -half; rx <= half; rx++) {
                    int px = x0 + rx, py = y0 + ry;
                    if (px >= 0 && px < MAP_CANVAS_W && py >= 0 && py < MAP_CANVAS_H)
                        s_canvas_buf[py * MAP_CANVAS_W + px] = color;
                }
            }
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // ── 绘制轨迹到 canvas buf ────────────────────────────
    void _drawTrack() {
        if (_trackCount < 1) return;

        lv_color_t track_color = lv_color_make(30, 144, 255);   // 道奇蓝
        lv_color_t dot_color   = lv_color_make(100, 200, 255);  // 浅蓝（历史点）

        int prev_cx = 0, prev_cy = 0;
        bool has_prev = false;

        for (int i = 0; i < _trackCount; i++) {
            int cx, cy;
            bool visible = _latLonToCanvasXY(_track[i].lat, _track[i].lon, cx, cy);

            if (has_prev && visible) {
                // 画连线
                _drawLine(prev_cx, prev_cy, cx, cy, track_color, 3);
            }

            // 画历史点（小圆点，半径 3）
            if (visible && i < _trackCount - 1) {
                for (int ry = -3; ry <= 3; ry++) {
                    for (int rx = -3; rx <= 3; rx++) {
                        if (rx * rx + ry * ry <= 9) {
                            int px = cx + rx, py = cy + ry;
                            if (px >= 0 && px < MAP_CANVAS_W && py >= 0 && py < MAP_CANVAS_H)
                                s_canvas_buf[py * MAP_CANVAS_W + px] = dot_color;
                        }
                    }
                }
            }

            if (visible) { prev_cx = cx; prev_cy = cy; has_prev = true; }
            else          { has_prev = false; }
        }
    }

    // 解码瓦片（JPEG 或 PNG）并写入 canvas 指定位置
    void _drawJpegToCanvas(uint8_t *data, size_t len, int ox, int oy) {
        s_tile_ox = ox;
        s_tile_oy = oy;
#ifdef TILE_IS_PNG
        // PNG 解码（pngle）
        pngle_t *pngle = pngle_new();
        if (pngle) {
            pngle_set_draw_callback(pngle, pngDrawCb);
            pngle_feed(pngle, data, len);
            pngle_destroy(pngle);
        }
#else
        // JPEG 解码（TJpgDec）
        TJpgDec.drawJpg(0, 0, data, len);
#endif
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

        // 应用 canvas 位置（使 GPS 点精确落在屏幕中心）
        if (canvas) {
            lv_obj_set_pos(canvas, _canvas_x, _canvas_y);
        }
        // marker 始终居中，不需要动态调整
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
