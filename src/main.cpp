#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "lv_conf.h"
#include <demos/lv_demos.h>
#include "TouchDrvCSTXXX.hpp"
#include <Wire.h>
#include <SD_MMC.h>
#include "Audio.h"
#include "es8311.h"

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

#define MAX_TOUCH_POINTS 5
int16_t x[MAX_TOUCH_POINTS];
int16_t y[MAX_TOUCH_POINTS];
TouchDrvCST92xx CST9217;
uint8_t touchAddress = 0x5A;

uint32_t screenWidth;
uint32_t screenHeight;

static lv_disp_draw_buf_t draw_buf;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_CO5300 *gfx = new Arduino_CO5300(
  bus, LCD_RESET /* RST */, 0 /* rotation */, LCD_WIDTH /* width */, LCD_HEIGHT /* height */, 6, 0, 0, 0);

Audio audio;
es8311_handle_t es_handle = NULL; // 全局 ES8311 句柄
#define EXAMPLE_VOICE_VOLUME 90

// Audio 回调函数
void audio_info(const char *info){
    Serial.print("audio_info: "); Serial.println(info);
    
    if (strstr(info, "SampleRate:") && es_handle) {
        int sampleRate = 0;
        if (sscanf(info, "SampleRate: %d", &sampleRate) == 1 && sampleRate > 0) {
            Serial.printf("Reconfiguring ES8311 for %dHz\n", sampleRate);
            const es8311_clock_config_t es_clk = {
                .mclk_inverted = false,
                .sclk_inverted = false,
                .mclk_from_mclk_pin = true,
                .mclk_frequency = sampleRate * 256,
                .sample_frequency = sampleRate
            };
            es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
            es8311_microphone_config(es_handle, false);
            es8311_voice_mute(es_handle, false);
            es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL);
        }
    }
}
void audio_id3data(const char *info){  
    Serial.print("id3data: "); Serial.println(info);
}
void audio_eof_mp3(const char *info){
    Serial.print("eof_mp3: "); Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station: "); Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle: "); Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate: "); Serial.println(info);
}
void audio_commercial(const char *info){
    Serial.print("commercial: "); Serial.println(info);
}
void audio_icyurl(const char *info){
    Serial.print("icyurl: "); Serial.println(info);
}
void audio_lasthost(const char *info){
    Serial.print("lasthost: "); Serial.println(info);
}


String listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.println("Listing directory: " + String(dirname));

  String dirContent = "Listing directory: " + String(dirname) + "\n";

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return "Failed to open directory\n";
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return "Not a directory\n";
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      String dirName = "  DIR : " + String(file.name()) + "\n";
      Serial.print(dirName);
      dirContent += dirName;
      if (levels) {
        dirContent += listDir(fs, file.path(), levels - 1);
      }
    } else {
      String fileInfo = "  FILE: " + String(file.name()) + "  SIZE: " + String(file.size()) + "\n";
      Serial.print(fileInfo);
      dirContent += fileInfo;
    }
    file = root.openNextFile();
  }
  return dirContent;
}
void audio_task(void *param) {
  delay(1000);
  
  Wire.begin(IIC_SDA, IIC_SCL);
  
  // 初始化 ES8311（使用 MCLK 引脚）
  es_handle = es8311_create(0, ES8311_ADDRRES_0);
  if (!es_handle) {
    Serial.println("ES8311 create failed!");
    vTaskDelete(NULL);
  }
  
  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = 44100 * 256,
    .sample_frequency = 44100
  };
  
  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));
  ESP_ERROR_CHECK(es8311_voice_mute(es_handle, false));
  ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL));
  
  Serial.println("ES8311 initialized");
  
  pinMode(PA, OUTPUT);
  digitalWrite(PA, HIGH);
  delay(100);
  
  // 配置 Audio I2S（MCLK 会自动输出）
  audio.setPinout(BCLKPIN, WSPIN, DOPIN, MCLKPIN);
  audio.setVolume(21);
  
  Serial.println("Playing /sdcard/1.mp3");
  audio.connecttoFS(SD_MMC, "/1.mp3");
  
  while (1) {
    audio.loop();
    vTaskDelay(1);
  }
}

#if LV_USE_LOG != 0
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area) {
  if(area->x1 % 2 != 0) area->x1--;
  if(area->y1 % 2 != 0) area->y1--;
  if(area->x2 % 2 == 0) area->x2++;
  if(area->y2 % 2 == 0) area->y2++;
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
  lv_disp_flush_ready(disp);
}

void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  uint8_t touched = CST9217.getPoint(x, y, CST9217.getSupportTouchPoint());
  if (touched > 0) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = x[0];
    data->point.y = y[0];
    Serial.printf("X:%d Y:%d\n", x[0], y[0]);
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  pinMode(PA, OUTPUT);
  digitalWrite(PA, HIGH);

  Wire.begin(IIC_SDA, IIC_SCL);
  CST9217.begin(Wire, touchAddress, IIC_SDA, IIC_SCL);
  CST9217.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
  CST9217.setMirrorXY(true, true);

  gfx->begin();
  gfx->setBrightness(200);
  screenWidth = gfx->width();
  screenHeight = gfx->height();

  lv_init();

  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * screenHeight / 4);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.rounder_cb = example_lvgl_rounder_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  // lv_obj_t *label = lv_label_create(lv_scr_act());
  // lv_label_set_text(label, "Hello Arduino and LVGL!");
  // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);

  lv_obj_t *sd_info_label = lv_label_create(lv_scr_act());
  lv_label_set_long_mode(sd_info_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(sd_info_label, 300);
  lv_obj_align(sd_info_label, LV_ALIGN_CENTER, 0, 0);

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Card Mount Failed");
    lv_label_set_text(sd_info_label, "Card Mount Failed");
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");
    lv_label_set_text(sd_info_label, "No SD_MMC card attached");
  }

  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.println("SD_MMC Card Size: " + String(cardSize) + "MB");

  char sd_info[256];
  snprintf(sd_info, sizeof(sd_info), "SD_MMC Card Type: %s\nSD_MMC Card Size: %lluMB\n",
           cardType == CARD_MMC ? "MMC" : cardType == CARD_SD   ? "SDSC"
                                        : cardType == CARD_SDHC ? "SDHC"
                                                                : "UNKNOWN",
           cardSize);

  String dirList = listDir(SD_MMC, "/", 0);
  strncat(sd_info, dirList.c_str(), sizeof(sd_info) - strlen(sd_info) - 1);

  lv_label_set_text(sd_info_label, sd_info);


  // lv_demo_widgets();  // 你也可以换成其他 demo

  xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 1, NULL, 1);

  Serial.println("Setup complete.");
}

void loop() {
  lv_timer_handler();
  delay(5);
}
