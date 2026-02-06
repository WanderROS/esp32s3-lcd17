#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "lv_conf.h"
#include <demos/lv_demos.h>
#include "TouchDrvCSTXXX.hpp"
#include <Wire.h>
#include "ESP_I2S.h"
#include <esp_vad.h>

#include "esp_check.h"
#include "es8311.h"
#include "es7210.h"
#include "ESP_SR.h"
#include "esp_partition.h"
#include "SD_MMC.h"

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

I2SClass i2s;
#define EXAMPLE_SAMPLE_RATE 16000
#define EXAMPLE_VOICE_VOLUME 90
#define EXAMPLE_ES8311_MIC_GAIN (es8311_mic_gain_t)(6)
#define EXAMPLE_ES7210_MIC_GAIN GAIN_30DB
#define RECORD_TIME_SEC 6
#define RECORD_BUFFER_SIZE (EXAMPLE_SAMPLE_RATE * RECORD_TIME_SEC)

static int16_t *record_buffer = NULL;
static volatile bool wake_detected = false;

esp_err_t es8311_codec_init(void) {
  es8311_handle_t es_handle = es8311_create(0, ES8311_ADDRRES_0);
  ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, "ES8311", "create failed");

  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256,
    .sample_frequency = EXAMPLE_SAMPLE_RATE
  };

  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_ERROR_CHECK(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency));
  ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));
  ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL));
  ESP_ERROR_CHECK(es8311_microphone_gain_set(es_handle, EXAMPLE_ES8311_MIC_GAIN));
  return ESP_OK;
}

void audio_task(void *param) {
  i2s.setPins(BCLKPIN, WSPIN, DIPIN, DOPIN, MCLKPIN);
  if (!i2s.begin(I2S_MODE_STD, EXAMPLE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("I2S init failed!");
    vTaskDelete(NULL);
  }

  Wire.begin(IIC_SDA, IIC_SCL);
  if (es8311_codec_init() != ESP_OK) {
    Serial.println("ES8311 init failed!");
    vTaskDelete(NULL);
  }

  audio_hal_codec_config_t es7210_cfg = {
    .adc_input = AUDIO_HAL_ADC_INPUT_ALL,
    .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,
    .codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE,
    .i2s_iface = {
      .mode = AUDIO_HAL_MODE_SLAVE,
      .fmt = AUDIO_HAL_I2S_NORMAL,
      .samples = AUDIO_HAL_16K_SAMPLES,
      .bits = AUDIO_HAL_BIT_LENGTH_16BITS
    }
  };

  if (es7210_adc_init(&Wire, &es7210_cfg) != ESP_OK) {
    Serial.println("ES7210 init failed!");
    vTaskDelete(NULL);
  }
  es7210_mic_select((es7210_input_mics_t)(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2));
  es7210_adc_set_gain_all(EXAMPLE_ES7210_MIC_GAIN);
  es7210_adc_ctrl_state(AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);
  Serial.println("ES7210 initialized");

  Serial.println("Checking model partition...");
  const esp_partition_t* model_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "model");
  if (!model_part) {
    Serial.println("ERROR: Model partition not found!");
    vTaskDelete(NULL);
  }
  Serial.printf("Model partition found at 0x%x, size: %d bytes\n", model_part->address, model_part->size);

  uint8_t test_buf[16];
  esp_partition_read(model_part, 0, test_buf, 16);
  Serial.print("First 16 bytes: ");
  for(int i=0; i<16; i++) Serial.printf("%02X ", test_buf[i]);
  Serial.println();

  vTaskDelay(pdMS_TO_TICKS(100));

  Serial.println("Initializing ESP_SR...");
  
  ESP_SR.onEvent([](sr_event_t event, int command_id, int phrase_id) {
  switch (event) {
    case SR_EVENT_WAKEWORD:
      Serial.println("WakeWord Detected!");  // Wakeword detected
      wake_detected = true;
      break;
    case SR_EVENT_WAKEWORD_CHANNEL:
      Serial.printf("WakeWord Channel %d Verified!\n", command_id);  // Specific wakeword channel verified
      ESP_SR.setMode(SR_MODE_COMMAND);                               // Switch to command detection mode
      break;
    case SR_EVENT_TIMEOUT:
      Serial.println("Timeout Detected!");  // Timeout occurred
      ESP_SR.setMode(SR_MODE_WAKEWORD);     // Switch back to wakeword detection mode
      break;
    case SR_EVENT_COMMAND:
      Serial.printf("Command %d \n", command_id);  // Command recognized
      switch (command_id) {
        default:
          Serial.println("Unknown Command!");  // Unknown command received
          break;
      }
      ESP_SR.setMode(SR_MODE_COMMAND);  // Allow for more commands to be given before timeout
      break;
    default:
      Serial.println("Unknown Event!");  // Unknown event received
      break;
  }});

  if (!ESP_SR.begin(i2s, NULL, 0, SR_CHANNELS_STEREO, SR_MODE_WAKEWORD)) {
    Serial.println("ESP_SR init failed!");
    vTaskDelete(NULL);
  }
  Serial.println("ESP_SR initialized, listening for '小美同学'...");

  record_buffer = (int16_t *)heap_caps_malloc(RECORD_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!record_buffer) {
    Serial.println("Record buffer allocation failed!");
    vTaskDelete(NULL);
  }

  while (1) {
    if (wake_detected) {
      wake_detected = false;
      
      ESP_SR.setMode(SR_MODE_OFF);
      vTaskDelay(pdMS_TO_TICKS(200));
      
      Serial.println("录音6秒...（单声道）");
      int16_t *stereo_buffer = (int16_t *)heap_caps_malloc(RECORD_BUFFER_SIZE * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
      uint32_t total_read = 0;
      uint32_t start_time = millis();
      while (total_read < RECORD_BUFFER_SIZE * 2 * sizeof(int16_t)) {
        size_t bytes_read = i2s.readBytes((char *)&stereo_buffer[total_read / sizeof(int16_t)], 
                                          (RECORD_BUFFER_SIZE * 2 * sizeof(int16_t) - total_read));
        total_read += bytes_read;
      }
      for (int i = 0; i < RECORD_BUFFER_SIZE; i++) {
        record_buffer[i] = stereo_buffer[i * 2];
      }
      heap_caps_free(stereo_buffer);
      uint32_t elapsed = millis() - start_time;
      Serial.printf("录音完成，耗时: %d ms, 读取: %d 字节\n", elapsed, total_read);
      
      char filename[32];
      snprintf(filename, sizeof(filename), "/%08lx.wav", esp_random());
      File file = SD_MMC.open(filename, FILE_WRITE);
      if (file) {
        uint32_t dataSize = RECORD_BUFFER_SIZE * sizeof(int16_t);
        uint32_t fileSize = dataSize + 36;
        file.write((uint8_t*)"RIFF", 4);
        file.write((uint8_t*)&fileSize, 4);
        file.write((uint8_t*)"WAVE", 4);
        file.write((uint8_t*)"fmt ", 4);
        uint32_t fmtSize = 16;
        uint16_t audioFormat = 1, channels = 1, bitsPerSample = 16;
        uint32_t sampleRate = EXAMPLE_SAMPLE_RATE, byteRate = sampleRate * channels * bitsPerSample / 8;
        uint16_t blockAlign = channels * bitsPerSample / 8;
        file.write((uint8_t*)&fmtSize, 4);
        file.write((uint8_t*)&audioFormat, 2);
        file.write((uint8_t*)&channels, 2);
        file.write((uint8_t*)&sampleRate, 4);
        file.write((uint8_t*)&byteRate, 4);
        file.write((uint8_t*)&blockAlign, 2);
        file.write((uint8_t*)&bitsPerSample, 2);
        file.write((uint8_t*)"data", 4);
        file.write((uint8_t*)&dataSize, 4);
        file.write((uint8_t*)record_buffer, dataSize);
        file.close();
        Serial.printf("已保存: %s\n", filename);
      } else {
        Serial.println("文件打开失败!");
      }
      
      Serial.println("播放...");
      int16_t *playback_buffer = (int16_t *)heap_caps_malloc(RECORD_BUFFER_SIZE * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
      for (int i = 0; i < RECORD_BUFFER_SIZE; i++) {
        int16_t sample = record_buffer[i] * 2;
        playback_buffer[i * 2] = sample;
        playback_buffer[i * 2 + 1] = sample;
      }
      i2s.write((uint8_t *)playback_buffer, RECORD_BUFFER_SIZE * 2 * sizeof(int16_t));
      heap_caps_free(playback_buffer);
      Serial.println("完成，继续监听...");
      
      vTaskDelay(pdMS_TO_TICKS(200));
      ESP_SR.setMode(SR_MODE_WAKEWORD);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
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
  delay(4000);
  pinMode(PA, OUTPUT);
  digitalWrite(PA, HIGH);

  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card init failed!");
  }

  Wire.begin(IIC_SDA, IIC_SCL);
  CST9217.begin(Wire, touchAddress, IIC_SDA, IIC_SCL);
  CST9217.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
  CST9217.setMirrorXY(true, true);

  gfx->begin();
  gfx->setBrightness(200);
  screenWidth = gfx->width();
  screenHeight = gfx->height();

  lv_init();

  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * 40 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  lv_color_t *buf2 = NULL;

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * 40);

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

  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello Arduino and LVGL!");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  // lv_demo_widgets();  // Disabled to save memory for ESP_SR

  Serial.printf("Free heap: %d, PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

  Serial.println("Delaying audio task to save memory during startup...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 5, NULL, 1);

  Serial.println("Setup complete.");
}

void loop() {
  lv_timer_handler();
  delay(5);
}
