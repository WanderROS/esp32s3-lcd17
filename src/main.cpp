#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "lv_conf.h"
#include <demos/lv_demos.h>
#include "TouchDrvCSTXXX.hpp"
#include <Wire.h>
#include "Audio.h"
#include "SD_MMC.h"
#include <cstdio>
#include "esp_check.h"
#include "es8311.h"
#include "XPowersLib.h"
#include "SensorPCF85063.hpp"
#include "SensorQMI8658.hpp"
#include "lvgl_sd_resource/lvgl_sd_resource.h"
#include "menu/menu.h"
#include <WiFi.h>
#include "esp_sntp.h"
#include "WiFiProv.h"
#include "esp_heap_caps.h"

// ---- 蓝牙配网参数 ----
const char *prov_pop          = "abcd1234";    // 配对PIN码，手机App中输入
const char *prov_service_name = "PROV_CANON";  // 蓝牙设备名，App中可见
const char *prov_service_key  = NULL;          // SoftAP密码，BLE模式下为NULL
// ----------------------

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

#define MAX_TOUCH_POINTS 5
int16_t x[MAX_TOUCH_POINTS];
int16_t y[MAX_TOUCH_POINTS];
TouchDrvCST92xx CST9217;
uint8_t touchAddress = 0x5A;

#include <ESP_IOExpander_Library.h>
ESP_IOExpander *expander = nullptr;
#define TCA9554_I2C_ADDRESS 0x20
#define PMU_IRQ_PIN 5

uint32_t screenWidth;
uint32_t screenHeight;

lv_obj_t *label;    // Global label object
lv_display_t *disp; // Global display object
SensorPCF85063 rtc;
uint32_t lastMillis;

// ---- WiFi 状态 ----
enum WifiStatus { WIFI_STATUS_DISCONNECTED, WIFI_STATUS_CONNECTED, WIFI_STATUS_BLINK };
static volatile WifiStatus wifi_status = WIFI_STATUS_DISCONNECTED;
static lv_obj_t *wifi_canvas = nullptr;
static lv_draw_buf_t wifi_draw_buf;
static uint8_t wifi_canvas_buf[LV_CANVAS_BUF_SIZE(36, 36, 32, 4)];

// ---- 图标栏：自动在圆形屏幕右上角从右往左排列 ----
// 图标栏：自动在圆形屏幕右上角从右往左排列
// 新增图标只需 iconbar_add()，不可见图标调用 iconbar_set_visible() 后再 iconbar_layout() 重排
#define ICONBAR_MAX 8
static struct {
  lv_obj_t *obj[ICONBAR_MAX];
  bool      visible[ICONBAR_MAX];
  int       count;
  int       icon_h;   // 所有图标统一高度
  float     y_ratio;  // 图标底部距圆心的比例（0~1，越大越靠上）
  int       margin;   // 图标右边缘距圆边缘的安全距离（px）
  int       gap;      // 图标之间的间距（px）
} iconbar = {
  {},     // obj[]
  {},     // visible[]
  0,      // count
  36,     // icon_h
  0.70f,  // y_ratio
  6,      // margin
  6,      // gap
};

static void iconbar_add(lv_obj_t *obj, bool visible = true) {
  if (iconbar.count < ICONBAR_MAX) {
    iconbar.obj[iconbar.count]     = obj;
    iconbar.visible[iconbar.count] = visible;
    iconbar.count++;
  }
}

// 更新某个图标的可见性并重新排列（不可见的图标移出屏幕外）
static void iconbar_set_visible(lv_obj_t *obj, bool visible) {
  for (int i = 0; i < iconbar.count; i++) {
    if (iconbar.obj[i] == obj) {
      iconbar.visible[i] = visible;
      break;
    }
  }
}

static void iconbar_layout(void) {
  int r   = screenWidth / 2;
  int cx  = screenWidth / 2;
  int cy  = screenHeight / 2;

  int icon_bottom_y = cy - (int)(r * iconbar.y_ratio);
  int icon_top_y    = icon_bottom_y - iconbar.icon_h;

  int dy_top    = cy - icon_top_y;
  int dy_bottom = cy - icon_bottom_y;
  int dy_worst  = (abs(dy_top) > abs(dy_bottom)) ? abs(dy_top) : abs(dy_bottom);
  int max_x     = cx + (int)sqrtf((float)(r*r - dy_worst*dy_worst)) - iconbar.margin;

  // 只排列可见图标，不可见的移到屏幕外
  int x = max_x;
  for (int i = 0; i < iconbar.count; i++) {
    if (iconbar.visible[i]) {
      x -= iconbar.icon_h;
      lv_obj_set_pos(iconbar.obj[i], x, icon_top_y);
      x -= iconbar.gap;
    } else {
      lv_obj_set_pos(iconbar.obj[i], -iconbar.icon_h, icon_top_y); // 移出屏幕
    }
  }
}

// ---- 电池图标 ----
static lv_obj_t      *batt_canvas = nullptr;
static lv_draw_buf_t  batt_draw_buf;
static uint8_t        batt_canvas_buf[LV_CANVAS_BUF_SIZE(36, 36, 32, 4)];

// percent: 0~100, -1=无电池; charging: 充电中; vbus: 已接电源
// 横向电池：主体在左，正极头在右
static void draw_batt_icon(int percent, bool charging, bool vbus) {
  if (!batt_canvas) return;
  lv_canvas_fill_bg(batt_canvas, lv_color_black(), LV_OPA_TRANSP);

  lv_layer_t layer;
  lv_canvas_init_layer(batt_canvas, &layer);

  // 横向布局，垂直居中
  // 主体外框: x=1~29, y=7~28  (宽29, 高22)
  // 正极头:   x=30~34, y=12~23 (宽5, 高12)
  const int bx = 1,  by = 7, bw = 29, bh = 22;
  const int tx = 30, ty = 12, tw = 5,  th = 12;

  lv_color_t frame_color;
  if (percent < 0)        frame_color = lv_color_hex(0x555555);
  else if (percent <= 15) frame_color = lv_color_hex(0xCC2222);
  else if (charging)      frame_color = lv_color_hex(0x00AAFF);
  else                    frame_color = lv_color_hex(0xAAAAAA);

  // 正极头（实心小块）
  lv_draw_rect_dsc_t cap; lv_draw_rect_dsc_init(&cap);
  cap.bg_color = frame_color; cap.bg_opa = LV_OPA_COVER;
  cap.radius = 1; cap.border_width = 0;
  lv_area_t cap_area = { tx, ty, tx+tw-1, ty+th-1 };
  lv_draw_rect(&layer, &cap, &cap_area);

  // 主体外框（空心）
  lv_draw_rect_dsc_t frame; lv_draw_rect_dsc_init(&frame);
  frame.bg_opa = LV_OPA_TRANSP;
  frame.border_color = frame_color; frame.border_width = 2;
  frame.border_opa = LV_OPA_COVER; frame.radius = 2;
  lv_area_t frame_area = { bx, by, bx+bw-1, by+bh-1 };
  lv_draw_rect(&layer, &frame, &frame_area);

  if (percent >= 0) {
    // 电量填充（从左往右）
    const int inner_y  = by + 3;
    const int inner_h  = bh - 6;
    const int inner_x  = bx + 3;
    const int inner_max_w = bw - 6;
    int fill_w = (inner_max_w * percent) / 100;
    if (fill_w < 1 && percent > 0) fill_w = 1;

    lv_color_t fill_color;
    if      (percent <= 15) fill_color = lv_color_hex(0xCC2222);
    else if (percent <= 40) fill_color = lv_color_hex(0xFF8800);
    else if (charging)      fill_color = lv_color_hex(0x00AAFF);
    else                    fill_color = lv_color_hex(0x00CC66);

    if (fill_w > 0) {
      lv_draw_rect_dsc_t fill; lv_draw_rect_dsc_init(&fill);
      fill.bg_color = fill_color; fill.bg_opa = LV_OPA_COVER;
      fill.radius = 1; fill.border_width = 0;
      lv_area_t fill_area = { inner_x, inner_y, inner_x + fill_w - 1, inner_y + inner_h - 1 };
      lv_draw_rect(&layer, &fill, &fill_area);
    }

    // 充电闪电：像素级 ⚡，横向电池中央
    // 形状：上横 → 左斜下 → 中横 → 左斜下 → 下横
    if (charging) {
      // 闪电中心
      int lx = bx + bw/2 - 1;
      int ly = by + bh/2;
      lv_draw_line_dsc_t b; lv_draw_line_dsc_init(&b);
      b.color = lv_color_white(); b.width = 2; b.opa = LV_OPA_COVER;
      // 上半：右上 → 左下（斜）
      b.p1.x=lx+4; b.p1.y=ly-4; b.p2.x=lx;   b.p2.y=ly;   lv_draw_line(&layer,&b);
      // 中横：向右突出
      b.p1.x=lx;   b.p1.y=ly;   b.p2.x=lx+3; b.p2.y=ly;   lv_draw_line(&layer,&b);
      // 下半：右 → 左下（斜）
      b.p1.x=lx+3; b.p1.y=ly;   b.p2.x=lx-1; b.p2.y=ly+4; lv_draw_line(&layer,&b);
    }
  } else {
    // 无电池：框内画横线表示缺失
    lv_draw_line_dsc_t q; lv_draw_line_dsc_init(&q);
    q.color = lv_color_hex(0x555555); q.width = 2; q.opa = LV_OPA_COVER;
    int mx = bx + bw/2, my = by + bh/2;
    q.p1.x=mx-5; q.p1.y=my; q.p2.x=mx+5; q.p2.y=my; lv_draw_line(&layer,&q);
    q.p1.x=mx;   q.p1.y=my-4; q.p2.x=mx; q.p2.y=my+4; lv_draw_line(&layer,&q);
  }

  lv_canvas_finish_layer(batt_canvas, &layer);
}

// ---- 铃声/功放图标 ----
static lv_obj_t      *spk_canvas = nullptr;
static lv_draw_buf_t  spk_draw_buf;
static uint8_t        spk_canvas_buf[LV_CANVAS_BUF_SIZE(36, 36, 32, 4)];
static bool           pa_enabled = true;  // 功放初始开启
static int            spk_volume = 1;     // 当前音量，与 audio.setVolume 同步

// ---- 音乐播放图标 ----
static lv_obj_t      *music_canvas = nullptr;
static lv_draw_buf_t  music_draw_buf;
static uint8_t        music_canvas_buf[LV_CANVAS_BUF_SIZE(36, 36, 32, 4)];

// 双音符图标，播放时显示蓝色，停止时清空
static void draw_music_icon(bool playing) {
  if (!music_canvas) return;
  lv_canvas_fill_bg(music_canvas, lv_color_black(), LV_OPA_TRANSP);
  if (!playing) return;

  lv_layer_t layer;
  lv_canvas_init_layer(music_canvas, &layer);

  lv_color_t c = lv_color_hex(0x00AAFF);
  lv_draw_line_dsc_t ln; lv_draw_line_dsc_init(&ln);
  ln.color = c; ln.opa = LV_OPA_COVER;

  // 音符1：符干
  ln.width=2; ln.p1.x=12; ln.p1.y=8; ln.p2.x=12; ln.p2.y=22; lv_draw_line(&layer,&ln);
  // 音符1：符头
  lv_draw_rect_dsc_t h; lv_draw_rect_dsc_init(&h);
  h.bg_color=c; h.bg_opa=LV_OPA_COVER; h.radius=3; h.border_width=0;
  lv_area_t a={7,20,14,25}; lv_draw_rect(&layer,&h,&a);

  // 音符2：符干
  ln.width=2; ln.p1.x=23; ln.p1.y=5; ln.p2.x=23; ln.p2.y=19; lv_draw_line(&layer,&ln);
  // 音符2：符头
  lv_area_t b={18,17,25,22}; lv_draw_rect(&layer,&h,&b);

  // 连接横梁
  ln.width=3; ln.p1.x=12; ln.p1.y=8; ln.p2.x=23; ln.p2.y=5; lv_draw_line(&layer,&ln);

  lv_canvas_finish_layer(music_canvas, &layer);
}

static void draw_spk_icon(bool enabled, int volume = 6) {
  if (!spk_canvas) return;
  lv_canvas_fill_bg(spk_canvas, lv_color_black(), LV_OPA_TRANSP);

  lv_layer_t layer;
  lv_canvas_init_layer(spk_canvas, &layer);

  lv_color_t c   = enabled ? lv_color_hex(0x00CC66) : lv_color_hex(0x444444);
  lv_color_t dim = lv_color_hex(0x2A2A2A);

  // 喇叭主体矩形
  lv_draw_rect_dsc_t body; lv_draw_rect_dsc_init(&body);
  body.bg_color = c; body.bg_opa = LV_OPA_COVER; body.radius = 1; body.border_width = 0;
  lv_area_t ba = {6, 14, 13, 22}; lv_draw_rect(&layer, &body, &ba);

  // 喇叭锥形三条线
  lv_draw_line_dsc_t ln; lv_draw_line_dsc_init(&ln);
  ln.color = c; ln.width = 2; ln.opa = LV_OPA_COVER;
  ln.p1.x=13; ln.p1.y=14; ln.p2.x=21; ln.p2.y=7;  lv_draw_line(&layer,&ln);
  ln.p1.x=13; ln.p1.y=22; ln.p2.x=21; ln.p2.y=29; lv_draw_line(&layer,&ln);
  ln.p1.x=21; ln.p1.y=7;  ln.p2.x=21; ln.p2.y=29; lv_draw_line(&layer,&ln);

  if (enabled) {
    // 音量分4档：0~5=0档, 6~10=1档, 11~15=2档, 16~21=3档
    // 声波弧线3条，按档位点亮
    int vol_bars;
    if      (volume <= 0)  vol_bars = 0;
    else if (volume <= 7)  vol_bars = 1;
    else if (volume <= 14) vol_bars = 2;
    else                   vol_bars = 3;

    // 3条弧，半径递增，圆心在喇叭口右侧
    static const int arc_radii[3] = { 6, 11, 16 };
    for (int i = 0; i < 3; i++) {
      lv_draw_arc_dsc_t arc; lv_draw_arc_dsc_init(&arc);
      arc.color       = (i < vol_bars) ? c : dim;
      arc.opa         = LV_OPA_COVER;
      arc.rounded     = 0;
      arc.width       = 2;
      arc.center.x    = 21;
      arc.center.y    = 18;
      arc.radius      = arc_radii[i];
      arc.start_angle = 330;
      arc.end_angle   = 30;
      lv_draw_arc(&layer, &arc);
    }
  } else {
    // 静音：红色 ✕
    lv_draw_line_dsc_t x; lv_draw_line_dsc_init(&x);
    x.color = lv_color_hex(0xCC2222); x.width = 2; x.opa = LV_OPA_COVER;
    x.p1.x=24; x.p1.y=10; x.p2.x=32; x.p2.y=26; lv_draw_line(&layer,&x);
    x.p1.x=32; x.p1.y=10; x.p2.x=24; x.p2.y=26; lv_draw_line(&layer,&x);
  }

  lv_canvas_finish_layer(spk_canvas, &layer);
}

static void draw_wifi_icon(int rssi, bool connected, bool blink_on) {
  if (!wifi_canvas) return;

  lv_canvas_fill_bg(wifi_canvas, lv_color_black(), LV_OPA_TRANSP);
  if (!connected && !blink_on) return;

  int bars;
  lv_color_t on_color;
  if (!connected) {
    bars = 1;
    on_color = lv_color_hex(0xFF6600);
  } else {
    on_color = lv_color_hex(0x00CC66);
    if      (rssi >= -55) bars = 4;
    else if (rssi >= -65) bars = 3;
    else if (rssi >= -75) bars = 2;
    else                  bars = 1;
  }

  const lv_color_t dim = lv_color_hex(0x383838);

  // 所有弧共享同一圆心和角度，只有半径不同
  // 圆心在底部中央，角度 225°~315°
  const int32_t cx = 18, cy = 30;
  const int16_t A1 = 225, A2 = 315;
  const int32_t radii[4] = { 6, 11, 16, 21 };

  lv_layer_t layer;
  lv_canvas_init_layer(wifi_canvas, &layer);

  for (int i = 0; i < 4; i++) {
    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.center.x    = cx;
    dsc.center.y    = cy;
    dsc.radius      = radii[i];
    dsc.start_angle = A1;
    dsc.end_angle   = A2;
    dsc.width       = 3;
    dsc.rounded     = 0;
    dsc.color       = (i < bars) ? on_color : dim;
    dsc.opa         = LV_OPA_COVER;
    lv_draw_arc(&layer, &dsc);
  }

  // 中心点
  lv_draw_rect_dsc_t dot;
  lv_draw_rect_dsc_init(&dot);
  dot.bg_color = on_color;
  dot.bg_opa   = LV_OPA_COVER;
  dot.radius   = LV_RADIUS_CIRCLE;
  lv_area_t dot_area = { cx - 2, cy - 2, cx + 2, cy + 2 };
  lv_draw_rect(&layer, &dot, &dot_area);

  lv_canvas_finish_layer(wifi_canvas, &layer);
}

SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;
float angleX = 1;
float angleY = 0;

bool rotation = false;
static uint8_t current_rotation = 3; // 0=0°, 1=90°, 2=180°, 3=270°

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET /* RST */, 0 /* rotation */, LCD_WIDTH /* width */, LCD_HEIGHT /* height */, 6, 0, 0, 0);

Audio audio;

#define EXAMPLE_SAMPLE_RATE 44100
#define EXAMPLE_VOICE_VOLUME 90
#define EXAMPLE_ES8311_MIC_GAIN (es8311_mic_gain_t)(3)
esp_err_t es8311_codec_init(void)
{
  es8311_handle_t es_handle = es8311_create(0, ES8311_ADDRRES_0);
  ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, "ES8311", "create failed");

  const es8311_clock_config_t es_clk = {
      .mclk_inverted = false,
      .sclk_inverted = false,
      .mclk_from_mclk_pin = true,
      .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256,
      .sample_frequency = EXAMPLE_SAMPLE_RATE};

  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_ERROR_CHECK(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency));
  ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));
  ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL));
  ESP_ERROR_CHECK(es8311_microphone_gain_set(es_handle, EXAMPLE_ES8311_MIC_GAIN));
  return ESP_OK;
}

void second_button_ck(lv_event_t *e)
{
  Serial.println("Second Clicked!");
  current_rotation = (current_rotation + 1) % 4;
  lv_display_rotation_t rotations[] = {LV_DISPLAY_ROTATION_0, LV_DISPLAY_ROTATION_90, LV_DISPLAY_ROTATION_180, LV_DISPLAY_ROTATION_270};
  lv_display_set_rotation(disp, rotations[current_rotation]);
}
std::vector<String> mp3Files;
int currentMp3Index = 0;

void audio_task(void *param)
{
  // SD卡已在setup()中初始化，这里不再重复
  if (!SD_MMC.cardType())
  {
    Serial.println("SD Card not available for audio!");
    vTaskDelete(NULL);
  }

  Wire.begin(IIC_SDA, IIC_SCL);
  if (es8311_codec_init() != ESP_OK)
  {
    Serial.println("ES8311 init failed!");
    vTaskDelete(NULL);
  }

  delay(100);

  audio.setPinout(BCLKPIN, WSPIN, DIPIN, MCLKPIN);
  audio.setVolume(spk_volume);

  delay(100);

  File root = SD_MMC.open("/");
  if (root)
  {
    File file = root.openNextFile();
    while (file)
    {
      String fileName = String(file.name());
      if (!file.isDirectory() &&
          (fileName.endsWith(".mp3") || fileName.endsWith(".MP3") ||
           fileName.endsWith(".wav") || fileName.endsWith(".WAV")))
      {
        mp3Files.push_back("/" + fileName);
        Serial.println("Found: " + fileName);
      }
      file = root.openNextFile();
    }
  }

  if (mp3Files.size() > 0)
  {
    audio.connecttoFS(SD_MMC, mp3Files[currentMp3Index].c_str());
    Serial.println("Playing: " + mp3Files[currentMp3Index]);
  }
  else
  {
    Serial.println("No MP3 files found!");
  }

  while (1)
  {
    audio.loop();
    vTaskDelay(1);
  }
}

#if LV_USE_LOG != 0
void my_print(int8_t level, const char *buf)
{
  Serial.printf(buf);
  Serial.flush();
}
#endif

// ========== 手动注册 LVGL 文件系统驱动 (基于 C stdio) ==========
static void *fs_open_cb(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode)
{
  const char *flags = (mode == LV_FS_MODE_WR) ? "wb" : "rb";
  char full_path[256];
  snprintf(full_path, sizeof(full_path), "/sdcard/%s", path);
  FILE *f = fopen(full_path, flags);
  return f;
}

static lv_fs_res_t fs_close_cb(lv_fs_drv_t *drv, void *file_p)
{
  fclose((FILE *)file_p);
  return LV_FS_RES_OK;
}

static lv_fs_res_t fs_read_cb(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br)
{
  *br = fread(buf, 1, btr, (FILE *)file_p);
  return (*br > 0 || btr == 0) ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
}

static lv_fs_res_t fs_seek_cb(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence)
{
  int w;
  switch (whence)
  {
  case LV_FS_SEEK_SET: w = SEEK_SET; break;
  case LV_FS_SEEK_CUR: w = SEEK_CUR; break;
  case LV_FS_SEEK_END: w = SEEK_END; break;
  default: return LV_FS_RES_INV_PARAM;
  }
  fseek((FILE *)file_p, pos, w);
  return LV_FS_RES_OK;
}

static lv_fs_res_t fs_tell_cb(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p)
{
  *pos_p = ftell((FILE *)file_p);
  return LV_FS_RES_OK;
}

void lv_fs_sd_init(void)
{
  static lv_fs_drv_t drv;
  lv_fs_drv_init(&drv);
  drv.letter = 'A';
  drv.open_cb = fs_open_cb;
  drv.close_cb = fs_close_cb;
  drv.read_cb = fs_read_cb;
  drv.seek_cb = fs_seek_cb;
  drv.tell_cb = fs_tell_cb;
  lv_fs_drv_register(&drv);
  Serial.println("LVGL FS driver 'A' registered.");
}

static uint8_t *rotated_buf = nullptr;

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
  lv_display_rotation_t rotation = lv_display_get_rotation(disp);
  lv_area_t rotated_area;

  if (rotation != LV_DISPLAY_ROTATION_0)
  {
    lv_color_format_t cf = lv_display_get_color_format(disp);
    rotated_area = *area;
    lv_display_rotate_area(disp, &rotated_area);

    uint32_t src_stride = lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
    uint32_t dest_stride = lv_draw_buf_width_to_stride(lv_area_get_width(&rotated_area), cf);

    int32_t src_w = lv_area_get_width(area);
    int32_t src_h = lv_area_get_height(area);
    uint32_t buf_size = dest_stride * lv_area_get_height(&rotated_area);

    if (!rotated_buf)
    {
      rotated_buf = (uint8_t *)heap_caps_malloc(screenWidth * screenHeight * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (!rotated_buf)
      {
        Serial.println("Failed to allocate rotation buffer!");
        lv_display_flush_ready(disp);
        return;
      }
    }

    lv_draw_sw_rotate(color_p, rotated_buf, src_w, src_h, src_stride, dest_stride, rotation, cf);

    area = &rotated_area;
    color_p = rotated_buf;
  }

  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
#endif
  lv_display_flush_ready(disp);
}

void example_increase_lvgl_tick(void *arg)
{
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
  uint8_t touched = CST9217.getPoint(x, y, CST9217.getSupportTouchPoint());
  if (touched > 0)
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = x[0];
    data->point.y = y[0];
    // Serial.printf("X:%d Y:%d\n", x[0], y[0]);
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

XPowersPMU power;

bool pmu_flag = false;
bool adc_switch = false;

void IRAM_ATTR pmuIrqHandler()
{
  pmu_flag = true;
}

void adcOn()
{
  power.enableTemperatureMeasure();
  // Enable internal ADC detection
  power.enableBattDetection();
  power.enableVbusVoltageMeasure();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();
}

void adcOff()
{
  power.disableTemperatureMeasure();
  // Enable internal ADC detection
  power.disableBattDetection();
  power.disableVbusVoltageMeasure();
  power.disableBattVoltageMeasure();
  power.disableSystemVoltageMeasure();
}

/* ---- 菜单项点击回调 ---- */
static void on_menu_settings(lv_event_t *e)
{
    Serial.println("Menu: 设置");
    // TODO: 在这里实现设置页面跳转
}

static void on_menu_music(lv_event_t *e)
{
    Serial.println("Menu: 音乐播放");
    // TODO: 在这里实现音乐播放页面跳转
}

static void on_menu_wifi(lv_event_t *e)
{
    Serial.println("Menu: WiFi");
    // TODO: 在这里实现 WiFi 配置页面
}

// ---- NTP 同步（WiFi 连接成功后调用）----
static void sync_ntp_and_rtc() {
  configTzTime("CST-8", "ntp.aliyun.com", "ntp1.aliyun.com", "ntp2.aliyun.com");
  struct tm timeinfo = {};
  uint32_t ntpStart = millis();
  while (!getLocalTime(&timeinfo, 1000) && millis() - ntpStart < 10000) {
    Serial.print(".");
  }
  Serial.println();
  if (timeinfo.tm_year > 100) {
    rtc.setDateTime(
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec
    );
    Serial.printf("RTC synced: %04d-%02d-%02d %02d:%02d:%02d\n",
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    Serial.println("NTP sync failed, RTC keeps existing time.");
  }
}

// ---- WiFiProv 事件回调（运行在独立 FreeRTOS 任务中）----
void SysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("\nWiFi connected, IP: ");
      Serial.println(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));
      wifi_status = WIFI_STATUS_CONNECTED;
      sync_ntp_and_rtc();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("\nWiFi disconnected.");
      wifi_status = WIFI_STATUS_BLINK;
      break;
    case ARDUINO_EVENT_PROV_START:
      Serial.println("\n配网已启动，请使用手机 App 输入 WiFi 信息");
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.printf("\n收到 WiFi 凭据 SSID: %s\n",
        (const char *)sys_event->event_info.prov_cred_recv.ssid);
      break;
    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.println("\n配网失败，请重置后重试");
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("\n配网成功");
      break;
    case ARDUINO_EVENT_PROV_END:
      Serial.println("\n配网流程结束");
      break;
    default:
      break;
  }
}

static void on_menu_weather(lv_event_t *e)
{
    Serial.println("Menu: 天气");
    // TODO: 在这里实现天气页面跳转
}

void setup()
{
  Serial.begin(115200);
  // delay(4000);
  pinMode(PA, OUTPUT);
  digitalWrite(PA, HIGH);

  // 蓝牙配网必须在其他外设初始化之前启动，确保内部 RAM 充足（BLE 需要 ~100KB）
  WiFi.onEvent(SysProvEvent);
  Serial.println("启动蓝牙配网，请打开 ESP BLE Prov App 扫描二维码或搜索设备");
  uint8_t uuid[16] = {0xb4,0xdf,0x5a,0x1c,0x3f,0x6b,0xf4,0xbf,0xea,0x4a,0x82,0x03,0x04,0x90,0x1a,0x02};
  WiFiProv.beginProvision(
    NETWORK_PROV_SCHEME_BLE,
    NETWORK_PROV_SCHEME_HANDLER_FREE_BLE,
    NETWORK_PROV_SECURITY_1,
    prov_pop, prov_service_name, prov_service_key,
    uuid,
    false  // false = 已配过则直接用 NVS 中的凭据，不重置
  );
  WiFiProv.printQR(prov_service_name, prov_pop, "ble");

  Wire.begin(IIC_SDA, IIC_SCL);

  // 初始化TCA9554扩展IO
  expander = new ESP_IOExpander_TCA95xx_8bit(I2C_NUM_0, TCA9554_I2C_ADDRESS, IIC_SCL, IIC_SDA);
  expander->init();
  expander->begin();

  // 配置扩展IO的5号引脚为输入，用于检测PMU中断
  expander->pinMode(PMU_IRQ_PIN, INPUT);

  bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);

  if (result == false)
  {
    Serial.println("PMU is not online...");
    while (1)
      delay(50);
  }
  Serial.println("PMU getID:" + String(power.getChipID(), HEX));

  if (!rtc.begin(Wire, IIC_SDA, IIC_SCL))
  {
    Serial.println("Failed to find PCF8563 - check your wiring!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("Found PCF8563 RTC");

  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL))
  {
    Serial.println("Failed to find QMI8658 - check your wiring!");
    while (1)
    {
      delay(1000);
    }
  }

  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
  qmi.enableAccelerometer();
  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS, SensorQMI8658::GYR_ODR_7174_4Hz, SensorQMI8658::LPF_MODE_0);
  qmi.enableGyroscope();

  //  设置系统电压过低保护
  //  范围: 2600~3300mV
  power.setSysPowerDownVoltage(3300);
  // 获取系统电压过低保护电压
  Serial.printf("System Power Down Voltage: %u mV\n", power.getSysPowerDownVoltage());

  // 设置 电源键开机时间 长按1秒开机
  const char *powerOn[] = {"128ms", "512ms", "1000ms", "2000ms"};
  power.setPowerKeyPressOnTime(XPOWERS_POWERON_1S);
  uint8_t opt = power.getPowerKeyPressOnTime();
  Serial.printf("PowerKeyPressOnTime: %s\n", powerOn[opt]);

  // 设置 电源键关机时间
  const char *powerOff[] = {"4", "6", "8", "10"};
  power.setPowerKeyPressOffTime(XPOWERS_POWEROFF_6S);
  opt = power.getPowerKeyPressOffTime();
  Serial.printf("PowerKeyPressOffTime: %s Second\n", powerOff[opt]);

  // 禁用 TS 引脚检测（适用于没有电池温度传感器的板子）
  // 否则会导致充电异常
  power.disableTSPinMeasure();

  // 设置充电指示LED模式 常亮
  power.setChargingLedMode(XPOWERS_CHG_LED_ON);

  // 设置充电截止电压为 4.1V
  power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V4);

  // 设置预充电电流 50mA
  power.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);

  // 设置恒流充电电流（根据你的电池容量调整，建议 200-500mA）
  power.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA); // 改为 500mA

  // 设置充电终止电流 25mA
  power.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

  power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  // Clear all interrupt flags
  power.clearIrqStatus();
  // Enable the required interrupt function
  power.enableIRQ(
      XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ          // POWER KEY
      | XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ  // CHARGE
      | XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ       // BATTERY
      | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ | XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ // 电源按键的上升沿 下降沿
  );

  adcOn();
  CST9217.begin(Wire, touchAddress, IIC_SDA, IIC_SCL);
  CST9217.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
  CST9217.setMirrorXY(true, true);

  gfx->begin();
  gfx->setBrightness(200);
  screenWidth = gfx->width();
  screenHeight = gfx->height();

  lv_init();

  // 注册自定义文件系统驱动（直接内联，避免链接问题）
  {
    static lv_fs_drv_t fs_drv;
    lv_fs_drv_init(&fs_drv);
    fs_drv.letter = 'A';
    fs_drv.open_cb = fs_open_cb;
    fs_drv.close_cb = fs_close_cb;
    fs_drv.read_cb = fs_read_cb;
    fs_drv.seek_cb = fs_seek_cb;
    fs_drv.tell_cb = fs_tell_cb;
    lv_fs_drv_register(&fs_drv);

    // 验证驱动是否注册成功
    lv_fs_drv_t *check = lv_fs_get_drv('A');
    if (check && check->open_cb)
    {
      Serial.println("LVGL FS driver 'A' registered OK, open_cb is set.");
    }
    else
    {
      Serial.printf("LVGL FS driver 'A' registration FAILED! drv=%p open_cb=%p\n", check, check ? (void *)check->open_cb : nullptr);
    }
  }

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  // 全屏缓冲，FULL 模式避免 partial 脏区计算不准导致的乱码
  // 466*466*2 ≈ 424KB，放 PSRAM 没问题
  const size_t buf_size = screenWidth * screenHeight * sizeof(lv_color_t);
  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if(!buf1 || !buf2) {
    Serial.println("Warning: PSRAM full-screen buffer alloc failed");
    if(buf1) { free(buf1); buf1 = nullptr; }
    if(buf2) { free(buf2); buf2 = nullptr; }
    // 降级：单缓冲 partial，可能有乱码但至少能跑
    const size_t fallback_size = screenWidth * 40 * sizeof(lv_color_t);
    buf1 = (lv_color_t *)heap_caps_malloc(fallback_size, MALLOC_CAP_DMA);
  }

  disp = lv_display_create(screenWidth, screenHeight);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_FULL);

  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_0); // LV_DISPLAY_ROTATION_180 LV_DISPLAY_ROTATION_0 work well

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

  // 初始化SD卡（需要在LVGL文件系统使用前挂载）
  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);
  if (!SD_MMC.begin("/sdcard", true, false, BOARD_MAX_SDMMC_FREQ, 20))
  {
    Serial.println("SD Card init failed!");
  }
  else
  {
    Serial.println("SD Card mounted.");
  }

  // 注意: 调试文件检查已移除，避免在字体加载前占用文件描述符
  // 如需调试，可在 lvgl_sd_resource_init 之后添加

  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &example_increase_lvgl_tick,
      .name = "lvgl_tick"};

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  // lv_demo_widgets(); // 你也可以换成其他 demo
  Serial.printf("Free heap: %u, Free DMA: %u, Free PSRAM: %u\n",
    ESP.getFreeHeap(),
    heap_caps_get_free_size(MALLOC_CAP_DMA),
    heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  lvgl_sd_resource_init("A:assets/");

  /* 创建 XML 生成的主屏幕（暂时屏蔽）*/
  // lv_obj_t *main_scr = main_page_create();
  // menu_create(main_scr, menu_items, sizeof(menu_items) / sizeof(menu_items[0]), chinese_24);

  /* 时钟测试屏幕 */
  lv_obj_t *clock_scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(clock_scr, lv_color_black(), 0);

  lv_obj_t *clock_label = lv_label_create(clock_scr);
  lv_obj_set_style_text_color(clock_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_label, geist_light_60, 0);
  lv_obj_align(clock_label, LV_ALIGN_CENTER, 0, -20);
  lv_label_set_text(clock_label, "00:00");
  lv_obj_set_name(clock_label, "clock_label");

  lv_obj_t *date_label = lv_label_create(clock_scr);
  lv_obj_set_style_text_color(date_label, lv_color_hex(0xAAAAAA), 0);
  lv_obj_set_style_text_font(date_label, geist_semibold_20, 0);
  lv_obj_align(date_label, LV_ALIGN_CENTER, 0, 40);
  lv_label_set_text(date_label, "2026-03-04");
  lv_obj_set_name(date_label, "date_label");

  // ---- WiFi 图标 canvas ----
  lv_draw_buf_init(&wifi_draw_buf, 36, 36, LV_COLOR_FORMAT_ARGB8888, LV_STRIDE_AUTO, wifi_canvas_buf, sizeof(wifi_canvas_buf));
  wifi_canvas = lv_canvas_create(clock_scr);
  lv_canvas_set_draw_buf(wifi_canvas, &wifi_draw_buf);

  // ---- 电池图标 canvas ----
  lv_draw_buf_init(&batt_draw_buf, 36, 36, LV_COLOR_FORMAT_ARGB8888, LV_STRIDE_AUTO, batt_canvas_buf, sizeof(batt_canvas_buf));
  batt_canvas = lv_canvas_create(clock_scr);
  lv_canvas_set_draw_buf(batt_canvas, &batt_draw_buf);

  // ---- 铃声图标 canvas ----
  lv_draw_buf_init(&spk_draw_buf, 36, 36, LV_COLOR_FORMAT_ARGB8888, LV_STRIDE_AUTO, spk_canvas_buf, sizeof(spk_canvas_buf));
  spk_canvas = lv_canvas_create(clock_scr);
  lv_canvas_set_draw_buf(spk_canvas, &spk_draw_buf);

  // ---- 音乐播放图标 canvas ----
  lv_draw_buf_init(&music_draw_buf, 36, 36, LV_COLOR_FORMAT_ARGB8888, LV_STRIDE_AUTO, music_canvas_buf, sizeof(music_canvas_buf));
  music_canvas = lv_canvas_create(clock_scr);
  lv_canvas_set_draw_buf(music_canvas, &music_draw_buf);

  // 图标栏：从右往左注册，iconbar_layout 自动计算坐标
  iconbar_add(batt_canvas);           // 最右
  iconbar_add(wifi_canvas);
  iconbar_add(music_canvas, false);   // 初始不可见，播放时自动出现
  iconbar_add(spk_canvas);
  
  iconbar_layout();

  draw_wifi_icon(0, false, true);
  draw_batt_icon(power.getBatteryPercent(), power.isCharging(), power.isVbusIn());
  draw_music_icon(false);
  draw_spk_icon(pa_enabled, spk_volume);


  lv_screen_load(clock_scr);

  xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 1, NULL, 1);

  Serial.println("Setup complete.");
}
char displayBuf[64];
static uint32_t last_clock_update = 0;

void loop()
{
  lv_timer_handler();
  delay(5);

  // 每秒更新一次时钟
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
    auto dt = rtc.getDateTime();
    lv_obj_t *scr = lv_screen_active();
    lv_obj_t *cl = lv_obj_get_child_by_name(scr, "clock_label");
    lv_obj_t *dl = lv_obj_get_child_by_name(scr, "date_label");
    if (cl) {
      snprintf(displayBuf, sizeof(displayBuf), "%02d:%02d:%02d", dt.getHour(), dt.getMinute(), dt.getSecond());
      lv_label_set_text(cl, displayBuf);
    }
    if (dl) {
      snprintf(displayBuf, sizeof(displayBuf), "%04d-%02d-%02d", dt.getYear(), dt.getMonth(), dt.getDay());
      lv_label_set_text(dl, displayBuf);
    }

    // ---- 更新 WiFi 图标 ----
    if (wifi_canvas) {
      int rssi = (wifi_status == WIFI_STATUS_CONNECTED) ? WiFi.RSSI() : -100;
      switch (wifi_status) {
        case WIFI_STATUS_CONNECTED:
          draw_wifi_icon(rssi, true, true);
          break;
        case WIFI_STATUS_BLINK:
          draw_wifi_icon(rssi, false, dt.getSecond() % 2 == 0);
          break;
        case WIFI_STATUS_DISCONNECTED:
        default:
          // 禁用态：暗色图标 + 红色斜线，圆心和角度与 draw_wifi_icon 完全一致
          lv_canvas_fill_bg(wifi_canvas, lv_color_black(), LV_OPA_TRANSP);
          {
            const lv_color_t dim = lv_color_hex(0x383838);
            const int32_t cx = 18, cy = 30;
            const int32_t radii[4] = { 6, 11, 16, 21 };
            lv_layer_t layer;
            lv_canvas_init_layer(wifi_canvas, &layer);
            for (int i = 0; i < 4; i++) {
              lv_draw_arc_dsc_t dsc; lv_draw_arc_dsc_init(&dsc);
              dsc.center.x=cx; dsc.center.y=cy;
              dsc.radius=radii[i]; dsc.start_angle=225; dsc.end_angle=315;
              dsc.width=3; dsc.rounded=0; dsc.color=dim; dsc.opa=LV_OPA_COVER;
              lv_draw_arc(&layer, &dsc);
            }
            lv_draw_line_dsc_t line; lv_draw_line_dsc_init(&line);
            line.color=lv_color_hex(0xCC2222); line.width=2; line.opa=LV_OPA_COVER;
            line.p1.x=4; line.p1.y=4; line.p2.x=32; line.p2.y=32;
            lv_draw_line(&layer, &line);
            lv_canvas_finish_layer(wifi_canvas, &layer);
          }
          break;
      }
    }

    // ---- 更新电池图标 ----
    if (batt_canvas) {
      int pct = power.isBatteryConnect() ? power.getBatteryPercent() : -1;
      draw_batt_icon(pct, power.isCharging(), power.isVbusIn());
    }

    // ---- 更新扬声器图标 ----
    if (spk_canvas) {
      draw_spk_icon(pa_enabled, spk_volume);
    }

    // ---- 更新音乐播放图标 ----
    if (music_canvas) {
      bool playing = audio.isRunning();
      static bool last_playing = false;
      if (playing != last_playing) {
        last_playing = playing;
        iconbar_set_visible(music_canvas, playing);
        iconbar_layout();
      }
      draw_music_icon(playing);
    }
  }

  // if (qmi.getDataReady())
  // {
  //   if (qmi.getAccelerometer(acc.x, acc.y, acc.z))
  //   {
  //     angleX = acc.x;
  //     angleY = acc.y;
  //     if (angleX > 0.8 && !rotation)
  //     {
  //       lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
  //       rotation = true;
  //        Serial.println("翻转屏幕1");
  //     }
  //     else if (angleX < -0.8 && !rotation)
  //     {
  //       lv_disp_set_rotation(disp, LV_DISP_ROT_180);
  //       rotation = true;
  //        Serial.println("翻转屏幕2");
  //     }
  //     else if (angleY < -0.8 && !rotation)
  //     {
  //       lv_disp_set_rotation(disp, LV_DISP_ROT_270);
  //       rotation = true;
  //        Serial.println("翻转屏幕3");
  //     }
  //     else if (angleY > 0.8 && !rotation)
  //     {
  //       lv_disp_set_rotation(disp, LV_DISP_ROT_90);
  //       rotation = true;
  //       Serial.println("翻转屏幕4");
  //     }
  //     if ((angleX <= 0.8 && angleX >= -0.8) && (angleY <= 0.8 && angleY >= -0.8))
  //     {
  //       rotation = false; // 允许重新执行旋转
  //     }
  //   }
  // }

  // 检测扩展IO的PMU中断引脚
  if (expander->digitalRead(PMU_IRQ_PIN) == LOW)
  {
    pmu_flag = true;
  }

  if (pmu_flag)
  {
    pmu_flag = false;
    uint32_t status = power.getIrqStatus();
    if (power.isPekeyShortPressIrq())
    {
      audio.pauseResume();
      Serial.printf("Audio %s\n", audio.isRunning() ? "resumed" : "paused");
    }
    if (power.isPekeyLongPressIrq())
    {
      Serial.println("Long Press: 清除配网信息，重启后重新配网...");
      // WiFi.disconnect(true, true);  // true, true = disconnect + erase stored credentials
      // delay(500);
      // ESP.restart();

    }
    if (power.isBatInsertIrq())
    {
      Serial.println("isBatInsert\n\n\n");
    }
    if (power.isBatRemoveIrq())
    {
      Serial.println("isBatRemove\n\n\n");
    }
    if (power.isBatChargeDoneIrq())
    {
      Serial.println("isBatChargeDone\n\n\n");
    }
    if (power.isBatChargeStartIrq())
    {
      Serial.println("isBatChargeStart\n\n\n");
    }
    if (power.isBatOverVoltageIrq())
    {
      Serial.println("isBatOverVoltage\n\n\n");
    }
    if (power.isPekeyNegativeIrq())
    {
      Serial.println("isPekeyNegative\n\n\n");
    }
    if (power.isPekeyPositiveIrq())
    {
      Serial.println("isPekeyPositive\n\n\n");
    }

    power.clearIrqStatus();
  }

  // 电源状态信息（已注释，需要时取消注释）
  // 注意：String拼接会消耗大量栈空间，避免在loop中频繁使用
  // Serial.println(info);
}

// optional
void audio_info(const char *info)
{
  Serial.print("info        ");
  Serial.println(info);
}
void audio_id3data(const char *info)
{ // id3 metadata
  Serial.print("id3data     ");
  Serial.println(info);
}
void audio_eof_mp3(const char *info)
{ // end of file
  Serial.print("eof_mp3     ");
  Serial.println(info);
  if (mp3Files.size() > 0)
  {
    currentMp3Index = (currentMp3Index + 1) % mp3Files.size();
    audio.connecttoFS(SD_MMC, mp3Files[currentMp3Index].c_str());
    Serial.println("Playing: " + mp3Files[currentMp3Index]);
  }
}
void audio_showstation(const char *info)
{
  Serial.print("station     ");
  Serial.println(info);
}
void audio_showstreaminfo(const char *info)
{
  Serial.print("streaminfo  ");
  Serial.println(info);
}
void audio_showstreamtitle(const char *info)
{
  Serial.print("streamtitle ");
  Serial.println(info);
}
void audio_bitrate(const char *info)
{
  Serial.print("bitrate     ");
  Serial.println(info);
}
void audio_commercial(const char *info)
{ // duration in sec
  Serial.print("commercial  ");
  Serial.println(info);
}
void audio_icyurl(const char *info)
{ // homepage
  Serial.print("icyurl      ");
  Serial.println(info);
}
void audio_lasthost(const char *info)
{ // stream URL played
  Serial.print("lasthost    ");
  Serial.println(info);
}
void audio_eof_speech(const char *info)
{
  Serial.print("eof_speech  ");
  Serial.println(info);
}