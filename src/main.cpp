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
#define EXAMPLE_ES8311_MIC_GAIN (es8311_mic_gain_t)(3)
#define EXAMPLE_ES7210_MIC_GAIN GAIN_37_5DB
#define VAD_FRAME_LENGTH_MS 30
#define VAD_BUFFER_LENGTH (VAD_FRAME_LENGTH_MS * EXAMPLE_SAMPLE_RATE / 1000)
#define RECORD_TIME_MS 3000
#define RECORD_BUFFER_SIZE (EXAMPLE_SAMPLE_RATE * RECORD_TIME_MS / 1000)
#define VOICE_THRESHOLD 500

int16_t *vad_buff;
int16_t *record_buff;
vad_handle_t vad_inst;
bool recording = false;
int record_index = 0;

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

  vad_inst = vad_create(VAD_MODE_3);
  vad_buff = (int16_t *)malloc(VAD_BUFFER_LENGTH * sizeof(int16_t));
  record_buff = (int16_t *)malloc(RECORD_BUFFER_SIZE * sizeof(int16_t));
  if (!vad_buff || !record_buff) {
    Serial.println("Memory allocation failed!");
    vTaskDelete(NULL);
  }

  Serial.println("VAD audio system started");
  int debug_count = 0;
  while (1) {
    size_t bytes_read = i2s.readBytes((char *)vad_buff, VAD_BUFFER_LENGTH * sizeof(int16_t));
    if (bytes_read > 0) {
      int16_t max_val = 0;
      for (int i = 0; i < VAD_BUFFER_LENGTH; i++) {
        if (abs(vad_buff[i]) > max_val) max_val = abs(vad_buff[i]);
      }
      
      if (debug_count++ < 5) {
        Serial.printf("Samples: %d %d %d %d\n", vad_buff[0], vad_buff[1], vad_buff[2], vad_buff[3]);
      }
      
      vad_state_t vad_state = vad_process(vad_inst, vad_buff, EXAMPLE_SAMPLE_RATE, VAD_FRAME_LENGTH_MS);
      bool voice_detected = (vad_state == VAD_SPEECH) || (max_val > VOICE_THRESHOLD);
      
      if (voice_detected && !recording) {
        Serial.println("Recording started...");
        recording = true;
        record_index = 0;
      }
      
      if (recording) {
        for (int i = 0; i < VAD_BUFFER_LENGTH && record_index < RECORD_BUFFER_SIZE; i++) {
          record_buff[record_index++] = vad_buff[i];
        }
        
        if (record_index >= RECORD_BUFFER_SIZE) {
          Serial.println("=== WAV HEX START ===");
          // Serial.print("52494646");
          // uint32_t fileSize = 36 + RECORD_BUFFER_SIZE * 2;
          // Serial.printf("%02X%02X%02X%02X", fileSize&0xFF, (fileSize>>8)&0xFF, (fileSize>>16)&0xFF, (fileSize>>24)&0xFF);
          // Serial.print("5741564566D74201000000010000803E0000007D00000200100064617461");
          // uint32_t dataSize = RECORD_BUFFER_SIZE * 2;
          // Serial.printf("%02X%02X%02X%02X", dataSize&0xFF, (dataSize>>8)&0xFF, (dataSize>>16)&0xFF, (dataSize>>24)&0xFF);
          // for (int i = 0; i < RECORD_BUFFER_SIZE; i++) {
          //   Serial.printf("%02X%02X", record_buff[i]&0xFF, (record_buff[i]>>8)&0xFF);
          // }
          Serial.println();
          Serial.println("=== WAV HEX END ===");
          
          i2s.write((uint8_t *)record_buff, RECORD_BUFFER_SIZE * sizeof(int16_t));
          Serial.println("Playback complete");
          recording = false;
        }
      }
    }
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

  lv_demo_widgets();  // 你也可以换成其他 demo

  xTaskCreatePinnedToCore(audio_task, "audio_task", 4096, NULL, 1, NULL, 1);

  Serial.println("Setup complete.");
}

void loop() {
  lv_timer_handler();
  delay(5);
}
