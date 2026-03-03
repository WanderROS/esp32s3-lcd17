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
  audio.setVolume(6);

  delay(100);

  File root = SD_MMC.open("/");
  if (root)
  {
    File file = root.openNextFile();
    while (file)
    {
      String fileName = String(file.name());
      if (!file.isDirectory() && fileName.endsWith(".mp3"))
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
  uint16_t year = 2026;
  uint8_t month = 2;
  uint8_t day = 6;
  uint8_t hour = 19;
  uint8_t minute = 53;
  uint8_t second = 41;

  rtc.setDateTime(year, month, day, hour, minute, second);

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
  power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

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

  // 全屏缓冲，放 PSRAM，避免双缓冲 partial 模式的脏区同步问题
  const size_t buf_size = screenWidth * screenHeight * sizeof(lv_color_t);
  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  lv_color_t *buf2 = NULL;
  if(!buf1) {
    Serial.println("Warning: PSRAM full-screen buffer alloc failed, falling back to partial");
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

  /* 创建 XML 生成的主屏幕 */
  lv_obj_t *main_scr = main_page_create();

  /* 在主屏幕上添加功能菜单列表 */
  static const menu_item_t menu_items[] = {
      {LV_SYMBOL_SETTINGS,  "设置",     on_menu_settings,  NULL},
      {LV_SYMBOL_AUDIO,     "音乐播放", on_menu_music,     NULL},
      {LV_SYMBOL_WIFI,      "WiFi",     on_menu_wifi,      NULL},
      {LV_SYMBOL_GPS,       "天气",     on_menu_weather,   NULL},
  };
  // menu_create(main_scr, menu_items, sizeof(menu_items) / sizeof(menu_items[0]), chinese_24);

  lv_screen_load(main_scr);

  xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 1, NULL, 1);

  Serial.println("Setup complete.");
}
char displayBuf[64];
void loop()
{
  lv_timer_handler();
  delay(5);

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

  if (millis() - lastMillis > 1000)
  {
    // Serial.println("accx:" + String(angleX) + " accy:" + String(angleY) + "\n");
    lastMillis = millis();
    RTC_DateTime datetime = rtc.getDateTime();
    // Serial.printf(" Year :");
    // Serial.print(datetime.getYear());
    // Serial.printf(" Month:");
    // Serial.print(datetime.getMonth());
    // Serial.printf(" Day :");
    // Serial.print(datetime.getDay());
    // Serial.printf(" Hour:");
    // Serial.print(datetime.getHour());
    // Serial.printf(" Minute:");
    // Serial.print(datetime.getMinute());
    // Serial.printf(" Sec :");
    // Serial.println(datetime.getSecond());

    memset(displayBuf, 0, sizeof(displayBuf));
    snprintf(displayBuf, sizeof(displayBuf), "%04d-%d-%d %d:%02d:%02d\0",
             datetime.getYear(), datetime.getMonth(), datetime.getDay(),
             datetime.getHour(), datetime.getMinute(), datetime.getSecond());

    // Serial.println(displayBuf);

    // Update label with current time
    // lv_label_set_text(label, displayBuf);
    // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  }

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
      current_rotation = (current_rotation + 1) % 4;
      lv_display_rotation_t rotations[] = {LV_DISPLAY_ROTATION_0, LV_DISPLAY_ROTATION_90, LV_DISPLAY_ROTATION_180, LV_DISPLAY_ROTATION_270};
      lv_display_set_rotation(disp, rotations[current_rotation]);
      Serial.printf("Rotation: %d degrees\n", current_rotation * 90);
    }
    if (power.isPekeyLongPressIrq())
    {
      Serial.println("Long Press Power Key\n\n\n");
      // power.shutdown();
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