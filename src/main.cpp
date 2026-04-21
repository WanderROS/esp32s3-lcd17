#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "lv_conf.h"
#include "TouchDrvCSTXXX.hpp"
#include <Wire.h>
#include "ESP_I2S.h"
#include "esp_check.h"
#include "es8311.h"
#include "es7210.h"
#include "ESP_SR.h"
#include "esp_partition.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "lv_fs_memfile.h" // LVGL 内存文件系统驱动
#include "ble_prov.h"
#include "wifi_config.h"   // 仅保留云服务 API Key 等配置
#include "aliyun_asr.h"
#include "qwen_llm.h"
#include "aliyun_tts.h"

// ===== 唤醒词命令（保留用于触发录音） =====
static const sr_cmd_t sr_commands[] = {};  // 无自定义命令，仅用唤醒词

// ===== 状态机 =====
enum VoiceState {
    STATE_IDLE,        // 等待唤醒
    STATE_RECORDING,   // 录音中
    STATE_ASR,         // 语音识别中
    STATE_LLM,         // 大模型推理中
    STATE_TTS,         // 语音合成+播放中
};

static volatile VoiceState voice_state = STATE_IDLE;
static volatile bool wake_detected = false;
static volatile bool g_provisioning = false;  // 配网进行中，延迟主界面切换

// ===== 音频配置 =====
#define EXAMPLE_SAMPLE_RATE     16000
#define EXAMPLE_VOICE_VOLUME    75  // 降低音量避免功放过驱动破音（范围 0~100）
#define EXAMPLE_ES8311_MIC_GAIN (es8311_mic_gain_t)(6)
#define EXAMPLE_ES7210_MIC_GAIN GAIN_30DB
#define RECORD_TIME_SEC         6
#define RECORD_BUFFER_SIZE      (EXAMPLE_SAMPLE_RATE * RECORD_TIME_SEC)

I2SClass i2s;
static int16_t *record_buffer = NULL;

// ===== LVGL 配置 =====
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define MAX_TOUCH_POINTS 5

int16_t touch_x[MAX_TOUCH_POINTS];
int16_t touch_y[MAX_TOUCH_POINTS];
TouchDrvCST92xx CST9217;
uint8_t touchAddress = 0x5A;
uint32_t screenWidth, screenHeight;
static lv_disp_draw_buf_t draw_buf;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);

// ===== UI 标签 =====
static lv_obj_t *lbl_status = NULL;
static lv_obj_t *lbl_asr    = NULL;
static lv_obj_t *lbl_reply  = NULL;

// 线程安全的 UI 更新（从任意任务调用）
static void ui_set_status(const char *text) {
    if (lbl_status) lv_label_set_text(lbl_status, text);
}
static void ui_set_asr(const char *text) {
    if (lbl_asr) lv_label_set_text(lbl_asr, text);
}
static void ui_set_reply(const char *text) {
    if (lbl_reply) lv_label_set_text(lbl_reply, text);
}

// ===== ES8311 初始化 =====
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

// ===== 录音函数（单声道，从立体声提取左声道）=====
bool do_record(int16_t *out_buf, size_t samples) {
    size_t stereo_bytes = samples * 2 * sizeof(int16_t);
    int16_t *stereo = (int16_t *)heap_caps_malloc(stereo_bytes, MALLOC_CAP_SPIRAM);
    if (!stereo) {
        Serial.println("[REC] 立体声缓冲区分配失败");
        return false;
    }

    size_t total_read = 0;
    uint32_t deadline = millis() + (RECORD_TIME_SEC + 2) * 1000;
    while (total_read < stereo_bytes && millis() < deadline) {
        size_t n = i2s.readBytes((char *)stereo + total_read, stereo_bytes - total_read);
        total_read += n;
    }

    // 提取左声道
    for (size_t i = 0; i < samples; i++) {
        out_buf[i] = stereo[i * 2];
    }
    heap_caps_free(stereo);
    Serial.printf("[REC] 录音完成，读取 %d bytes\n", total_read);
    return total_read > 0;
}

// ===== 主音频任务（运行在 Core 1）=====
void audio_task(void *param) {
    // --- 初始化 I2S ---
    i2s.setPins(BCLKPIN, WSPIN, DIPIN, DOPIN, MCLKPIN);
    if (!i2s.begin(I2S_MODE_STD, EXAMPLE_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                   I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
        Serial.println("[AUDIO] I2S 初始化失败!");
        vTaskDelete(NULL);
    }

    Wire.begin(IIC_SDA, IIC_SCL);
    if (es8311_codec_init() != ESP_OK) {
        Serial.println("[AUDIO] ES8311 初始化失败!");
        vTaskDelete(NULL);
    }

    audio_hal_codec_config_t es7210_cfg = {
        .adc_input  = AUDIO_HAL_ADC_INPUT_ALL,
        .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,
        .codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE,
        .i2s_iface  = {
            .mode    = AUDIO_HAL_MODE_SLAVE,
            .fmt     = AUDIO_HAL_I2S_NORMAL,
            .samples = AUDIO_HAL_16K_SAMPLES,
            .bits    = AUDIO_HAL_BIT_LENGTH_16BITS
        }
    };
    if (es7210_adc_init(&Wire, &es7210_cfg) != ESP_OK) {
        Serial.println("[AUDIO] ES7210 初始化失败!");
        vTaskDelete(NULL);
    }
    es7210_mic_select((es7210_input_mics_t)(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2));
    es7210_adc_set_gain_all(EXAMPLE_ES7210_MIC_GAIN);
    es7210_adc_ctrl_state(AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    // --- 检查模型分区 ---
    const esp_partition_t *model_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "model");
    if (!model_part) {
        Serial.println("[AUDIO] 模型分区未找到!");
        vTaskDelete(NULL);
    }
    Serial.printf("[AUDIO] 模型分区: 0x%x, %d bytes\n", model_part->address, model_part->size);

    vTaskDelay(pdMS_TO_TICKS(100));

    // 免唤醒窗口截止时间（在 lambda 里访问需要用全局变量）
    static uint32_t s_free_talk_until = 0;

    // --- 初始化 ESP_SR 唤醒词检测 ---
    ESP_SR.onEvent([](sr_event_t event, int command_id, int phrase_id) {
        switch (event) {
            case SR_EVENT_WAKEWORD:
                Serial.println("[SR] 唤醒词检测到!");
                // 免唤醒窗口内：第一阶段就触发，响应更快
                if (voice_state == STATE_IDLE && millis() < s_free_talk_until) {
                    wake_detected = true;
                    Serial.println("[SR] 免唤醒窗口内直接触发");
                }
                break;
            case SR_EVENT_WAKEWORD_CHANNEL:
                Serial.printf("[SR] 唤醒词通道 %d 确认!\n", command_id);
                if (voice_state == STATE_IDLE) {
                    wake_detected = true;
                }
                break;
            case SR_EVENT_TIMEOUT:
                Serial.println("[SR] 超时，返回唤醒模式");
                ESP_SR.setMode(SR_MODE_WAKEWORD);
                break;
            default:
                break;
        }
    });

    if (!ESP_SR.begin(i2s, sr_commands, 0, SR_CHANNELS_STEREO, SR_MODE_WAKEWORD)) {
        Serial.println("[SR] ESP_SR 初始化失败!");
        vTaskDelete(NULL);
    }
    Serial.println("[SR] 等待唤醒词 '小爱同学'...");
    ui_set_status("等待唤醒...");

    // --- 分配录音缓冲区 ---
    record_buffer = (int16_t *)heap_caps_malloc(
        RECORD_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!record_buffer) {
        Serial.println("[AUDIO] 录音缓冲区分配失败!");
        vTaskDelete(NULL);
    }

    // ===== 主循环 =====
    #define FREE_TALK_TIMEOUT_MS 30000  // 免唤醒窗口 30 秒
    // s_free_talk_until 已在上方声明为 static

    while (1) {
        bool in_free_talk = (millis() < s_free_talk_until);

        if (wake_detected) {
            wake_detected = false;
            voice_state = STATE_RECORDING;

            // 1. 停止唤醒词检测，开始录音
            ESP_SR.setMode(SR_MODE_OFF);
            vTaskDelay(pdMS_TO_TICKS(100));

            Serial.println("\n===== 开始录音 =====");
            ui_set_status("录音中...");
            ui_set_asr("");
            ui_set_reply("");

            bool rec_ok = do_record(record_buffer, RECORD_BUFFER_SIZE);

            if (!rec_ok) {
                Serial.println("[REC] 录音失败，返回唤醒模式");
                ui_set_status("录音失败，重试...");
                s_free_talk_until = 0;
                voice_state = STATE_IDLE;
                ESP_SR.setMode(SR_MODE_WAKEWORD);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            // 2. ASR 语音识别
            voice_state = STATE_ASR;
            Serial.println("===== ASR 识别 =====");
            ui_set_status("识别中...");

            String asr_text;
            bool asr_ok = aliyun_asr_recognize(record_buffer, RECORD_BUFFER_SIZE, asr_text);

            if (!asr_ok || asr_text.isEmpty()) {
                Serial.println("[ASR] 识别失败或无内容");
                // 免唤醒窗口内无内容 = 用户不想继续，退出免唤醒
                s_free_talk_until = 0;
                ui_set_status("等待唤醒...");
                voice_state = STATE_IDLE;
                ESP_SR.setMode(SR_MODE_WAKEWORD);
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }

            Serial.printf("[ASR] 识别: %s\n", asr_text.c_str());
            ui_set_asr(asr_text.c_str());

            // 3. 通义千问大模型
            voice_state = STATE_LLM;
            Serial.println("===== 大模型推理 =====");
            ui_set_status("思考中...");

            String llm_reply;
            bool llm_ok = qwen_chat(asr_text, llm_reply);

            if (!llm_ok || llm_reply.isEmpty()) {
                Serial.println("[LLM] 推理失败");
                ui_set_status("网络错误，重试...");
                aliyun_tts_speak("抱歉，网络出现问题，请稍后再试。");
                s_free_talk_until = 0;
                voice_state = STATE_IDLE;
                ESP_SR.setMode(SR_MODE_WAKEWORD);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            Serial.printf("[LLM] 回复: %s\n", llm_reply.c_str());
            ui_set_reply(llm_reply.c_str());

            // 4. TTS 语音合成 + 播放
            voice_state = STATE_TTS;
            Serial.println("===== TTS 播放 =====");
            ui_set_status("播放中...");

            aliyun_tts_speak(llm_reply);

            // 5. 完成：刷新免唤醒窗口，直接触发下一轮录音
            s_free_talk_until = millis() + FREE_TALK_TIMEOUT_MS;
            Serial.printf("[SR] 免唤醒窗口激活，剩余 %lu ms\n",
                          s_free_talk_until - millis());
            ui_set_status("继续说话 (30s)...");
            voice_state = STATE_IDLE;
            ESP_SR.setMode(SR_MODE_WAKEWORD);
            // 等待 TTS 播放声音消散，避免录到自己的声音
            vTaskDelay(pdMS_TO_TICKS(800));
            // 免唤醒：直接进入下一轮录音，无需唤醒词
            wake_detected = true;
        }

        // 免唤醒窗口倒计时提示（每5秒更新一次状态栏）
        if (s_free_talk_until > 0 && voice_state == STATE_IDLE) {
            uint32_t now = millis();
            if (now >= s_free_talk_until) {
                s_free_talk_until = 0;
                ui_set_status("等待唤醒...");
                Serial.println("[SR] 免唤醒窗口已过期");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===== LVGL 回调 =====
#if LV_USE_LOG != 0
void my_print(const char *buf) { Serial.print(buf); }
#endif

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area) {
    if (area->x1 % 2 != 0) area->x1--;
    if (area->y1 % 2 != 0) area->y1--;
    if (area->x2 % 2 == 0) area->x2++;
    if (area->y2 % 2 == 0) area->y2++;
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
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
    uint8_t touched = CST9217.getPoint(touch_x, touch_y, CST9217.getSupportTouchPoint());
    if (touched > 0) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ===== 字体相关 =====
static lv_font_t *g_font_cn_16 = nullptr;
static lv_font_t *g_font_cn_20 = nullptr;
static lv_font_t *g_font_cn_24 = nullptr;

// 需要在字体加载后更新字体的标签（全局保存引用）
static lv_obj_t *g_lbl_title       = nullptr;
static lv_obj_t *g_lbl_asr_title   = nullptr;
static lv_obj_t *g_lbl_reply_title = nullptr;
static lv_obj_t *g_lbl_hint        = nullptr;
static lv_obj_t *g_main_scr        = nullptr;  // 主界面屏幕对象（配网时延迟切换）

// 配网界面标签引用（字体加载完后更新）
static lv_obj_t *g_prov_lbl_title = nullptr;
static lv_obj_t *g_prov_lbl_hint  = nullptr;
static lv_obj_t *g_prov_lbl_name  = nullptr;

static const lv_font_t *cn_font(lv_font_t *cn, const lv_font_t *fallback) {
    return cn ? cn : fallback;
}

// 字体加载完成后，切换到主 UI 屏幕
static void apply_cn_fonts(void) {
    // 创建主 UI 屏幕（替换 Loading 屏）
    lv_obj_t *main_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(main_scr, lv_color_hex(0x1a1a2e), 0);

    g_lbl_title = lv_label_create(main_scr);
    lv_label_set_text(g_lbl_title, "AI \xe8\xaf\xad\xe9\x9f\xb3\xe5\x8a\xa9\xe6\x89\x8b");  // "AI 语音助手"
    lv_obj_set_style_text_color(g_lbl_title, lv_color_hex(0x00d4ff), 0);
    lv_obj_set_style_text_font(g_lbl_title, g_font_cn_24 ? g_font_cn_24 : &lv_font_montserrat_24, 0);
    lv_obj_align(g_lbl_title, LV_ALIGN_TOP_MID, 0, 20);

    lbl_status = lv_label_create(main_scr);
    lv_label_set_text(lbl_status, "\xe7\xad\x89\xe5\xbe\x85\xe5\x94\xa4\xe9\x86\x92...");  // "等待唤醒..."
    lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xffd700), 0);
    lv_obj_set_style_text_font(lbl_status, g_font_cn_20 ? g_font_cn_20 : &lv_font_montserrat_18, 0);
    lv_obj_align(lbl_status, LV_ALIGN_TOP_MID, 0, 60);

    lv_obj_t *line = lv_obj_create(main_scr);
    lv_obj_set_size(line, 400, 2);
    lv_obj_set_style_bg_color(line, lv_color_hex(0x444466), 0);
    lv_obj_set_style_border_width(line, 0, 0);
    lv_obj_align(line, LV_ALIGN_TOP_MID, 0, 100);

    g_lbl_asr_title = lv_label_create(main_scr);
    lv_label_set_text(g_lbl_asr_title, "\xe4\xbd\xa0\xe8\xaf\xb4\xef\xbc\x9a");  // "你说："
    lv_obj_set_style_text_color(g_lbl_asr_title, lv_color_hex(0x88aaff), 0);
    lv_obj_set_style_text_font(g_lbl_asr_title, g_font_cn_16 ? g_font_cn_16 : &lv_font_montserrat_16, 0);
    lv_obj_align(g_lbl_asr_title, LV_ALIGN_TOP_LEFT, 30, 115);

    lbl_asr = lv_label_create(main_scr);
    lv_label_set_text(lbl_asr, "");
    lv_label_set_long_mode(lbl_asr, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_asr, 400);
    lv_obj_set_style_text_color(lbl_asr, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(lbl_asr, g_font_cn_16 ? g_font_cn_16 : &lv_font_montserrat_16, 0);
    lv_obj_align(lbl_asr, LV_ALIGN_TOP_LEFT, 30, 140);

    g_lbl_reply_title = lv_label_create(main_scr);
    lv_label_set_text(g_lbl_reply_title, "AI\xef\xbc\x9a");  // "AI："
    lv_obj_set_style_text_color(g_lbl_reply_title, lv_color_hex(0x88ffaa), 0);
    lv_obj_set_style_text_font(g_lbl_reply_title, g_font_cn_16 ? g_font_cn_16 : &lv_font_montserrat_16, 0);
    lv_obj_align(g_lbl_reply_title, LV_ALIGN_TOP_LEFT, 30, 250);

    lbl_reply = lv_label_create(main_scr);
    lv_label_set_text(lbl_reply, "");
    lv_label_set_long_mode(lbl_reply, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_reply, 400);
    lv_obj_set_style_text_color(lbl_reply, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(lbl_reply, g_font_cn_16 ? g_font_cn_16 : &lv_font_montserrat_16, 0);
    lv_obj_align(lbl_reply, LV_ALIGN_TOP_LEFT, 30, 275);

    g_lbl_hint = lv_label_create(main_scr);
    lv_label_set_text(g_lbl_hint, "\xe8\xaf\xb4 '\xe5\xb0\x8f\xe7\x88\xb1\xe5\x90\x8c\xe5\xad\xa6' \xe5\x94\xa4\xe9\x86\x92");  // "说 '小爱同学' 唤醒"
    lv_obj_set_style_text_color(g_lbl_hint, lv_color_hex(0x666688), 0);
    lv_obj_set_style_text_font(g_lbl_hint, g_font_cn_16 ? g_font_cn_16 : &lv_font_montserrat_14, 0);
    lv_obj_align(g_lbl_hint, LV_ALIGN_BOTTOM_MID, 0, -55);

    // 重置配网按钮（右下角）
    lv_obj_t *btn_reset = lv_btn_create(main_scr);
    lv_obj_set_size(btn_reset, 120, 36);
    lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(btn_reset, lv_color_hex(0x3a1a1a), 0);
    lv_obj_set_style_bg_color(btn_reset, lv_color_hex(0x7a2020), LV_STATE_PRESSED);
    lv_obj_set_style_border_color(btn_reset, lv_color_hex(0x884444), 0);
    lv_obj_set_style_border_width(btn_reset, 1, 0);
    lv_obj_set_style_radius(btn_reset, 8, 0);
    lv_obj_add_event_cb(btn_reset, [](lv_event_t *e) {
        if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
            Serial.println("[UI] 重置配网，清除凭证并重启...");
            ui_set_status("\xe9\x87\x8d\xe7\xbd\xae\xe9\x85\x8d\xe7\xbd\x91...");  // "重置配网..."
            ble_prov_clear_credentials();
        }
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *btn_lbl = lv_label_create(btn_reset);
    lv_label_set_text(btn_lbl, "\xe9\x87\x8d\xe7\xbd\xae\xe9\x85\x8d\xe7\xbd\x91");  // "重置配网"
    lv_obj_set_style_text_color(btn_lbl, lv_color_hex(0xff8888), 0);
    lv_obj_set_style_text_font(btn_lbl, g_font_cn_16 ? g_font_cn_16 : &lv_font_montserrat_14, 0);
    lv_obj_center(btn_lbl);

    // 切换到主屏幕（带淡入动画）
    // 若配网仍在进行，先保存引用，等配网完成后再切换
    g_main_scr = main_scr;

    // 若配网界面正在显示，更新其字体
    if (g_prov_lbl_title)
        lv_obj_set_style_text_font(g_prov_lbl_title, cn_font(g_font_cn_24, &lv_font_montserrat_24), 0);
    if (g_prov_lbl_hint)
        lv_obj_set_style_text_font(g_prov_lbl_hint,  cn_font(g_font_cn_16, &lv_font_montserrat_14), 0);
    if (g_prov_lbl_name)
        lv_obj_set_style_text_font(g_prov_lbl_name,  cn_font(g_font_cn_20, &lv_font_montserrat_18), 0);

    if (!g_provisioning) {
        lv_scr_load_anim(main_scr, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, true);
        Serial.println("[FONT] 主界面已显示");
    } else {
        Serial.println("[FONT] 主界面已就绪，等待配网完成后切换");
    }
}

// 后台字体加载任务（Core 0，低优先级，不阻塞 UI）
static void font_load_task(void *param) {
    uint32_t t = millis();
    g_font_cn_24 = load_font_from_flash("/fonts/cn24.bin", 'A');
    g_font_cn_20 = load_font_from_flash("/fonts/cn20.bin", 'B');
    g_font_cn_16 = load_font_from_flash("/fonts/cn16.bin", 'C');
    Serial.printf("[FONT] 全部加载完成，总耗时 %lu ms\n", millis() - t);
    // 通过 LVGL 定时器在主任务上下文中安全更新 UI
    lv_timer_t *tmr = lv_timer_create([](lv_timer_t *t) {
        apply_cn_fonts();
        lv_timer_del(t);
    }, 10, nullptr);
    (void)tmr;
    vTaskDelete(NULL);
}

// ===== 配网二维码界面 =====
// 在 BLE 配网启动后调用，显示二维码和设备名
static void show_prov_qr_screen(const char *qr_payload, const char *dev_name) {
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    // 标题 "BLE 配网"
    g_prov_lbl_title = lv_label_create(scr);
    lv_label_set_text(g_prov_lbl_title, "BLE \xe9\x85\x8d\xe7\xbd\x91");
    lv_obj_set_style_text_color(g_prov_lbl_title, lv_color_hex(0x00d4ff), 0);
    lv_obj_set_style_text_font(g_prov_lbl_title,
        cn_font(g_font_cn_24, &lv_font_montserrat_24), 0);
    lv_obj_align(g_prov_lbl_title, LV_ALIGN_TOP_MID, 0, 18);

    // 二维码控件（白底黑码，200x200）
    lv_obj_t *qr = lv_qrcode_create(scr, 200, lv_color_hex(0x000000), lv_color_hex(0xffffff));
    lv_qrcode_update(qr, qr_payload, strlen(qr_payload));
    lv_obj_align(qr, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_style_border_color(qr, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_border_width(qr, 6, 0);

    // 提示文字 "扫码配网"
    g_prov_lbl_hint = lv_label_create(scr);
    lv_label_set_text(g_prov_lbl_hint,
        "ESP BLE Provisioning App \xe6\x89\xab\xe7\xa0\x81\xe9\x85\x8d\xe7\xbd\x91");
    lv_obj_set_style_text_color(g_prov_lbl_hint, lv_color_hex(0xaaaacc), 0);
    lv_obj_set_style_text_font(g_prov_lbl_hint,
        cn_font(g_font_cn_16, &lv_font_montserrat_14), 0);
    lv_obj_align(g_prov_lbl_hint, LV_ALIGN_BOTTOM_MID, 0, -40);

    // 设备名
    g_prov_lbl_name = lv_label_create(scr);
    lv_label_set_text(g_prov_lbl_name, dev_name);
    lv_obj_set_style_text_color(g_prov_lbl_name, lv_color_hex(0xffd700), 0);
    lv_obj_set_style_text_font(g_prov_lbl_name,
        cn_font(g_font_cn_20, &lv_font_montserrat_18), 0);
    lv_obj_align(g_prov_lbl_name, LV_ALIGN_BOTTOM_MID, 0, -16);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, true);
    Serial.println("[UI] 配网二维码界面已显示");
}

static void create_ui(void) {
    // Loading 屏：字体加载完成前显示，全用 Montserrat（无需中文字体）
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);

    lv_obj_t *lbl = lv_label_create(scr);
    lv_label_set_text(lbl, "Loading...");
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *lbl_sub = lv_label_create(scr);
    lv_label_set_text(lbl_sub, "Loading fonts from SD card");
    lv_obj_set_style_text_color(lbl_sub, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(lbl_sub, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_sub, LV_ALIGN_CENTER, 0, 20);

    // 后台异步加载字体（Core 0，优先级 2）
    xTaskCreatePinnedToCore(font_load_task, "font_load", 4096, NULL, 2, NULL, 0);
}

// ===== setup =====
void setup() {
    Serial.begin(115200);
    delay(2000);

    // 功放使能
    pinMode(PA, OUTPUT);
    digitalWrite(PA, HIGH);

    // SPIFFS 初始化（字体文件存储在 Flash）
    if (!SPIFFS.begin(true)) {
        Serial.println("[SPIFFS] 初始化失败!");
    } else {
        Serial.println("[SPIFFS] 初始化成功");
    }

    // 触摸初始化
    Wire.begin(IIC_SDA, IIC_SCL);
    CST9217.begin(Wire, touchAddress, IIC_SDA, IIC_SCL);
    CST9217.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
    CST9217.setMirrorXY(true, true);

    // 显示初始化
    gfx->begin();
    gfx->setBrightness(200);
    screenWidth  = gfx->width();
    screenHeight = gfx->height();

    // LVGL 初始化
    lv_init();
#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print);
#endif

    // draw buffer 必须在内部 RAM（DMA 可访问），不能用 PSRAM
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(
        screenWidth * 40 * sizeof(lv_color_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, screenWidth * 40);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res    = screenWidth;
    disp_drv.ver_res    = screenHeight;
    disp_drv.flush_cb   = my_disp_flush;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;
    disp_drv.draw_buf   = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // LVGL tick 定时器
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name     = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
    esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

    // 创建 UI
    create_ui();

    // 连接 WiFi（BLE 配网 或 NVS 已保存凭证）
    // 首次使用：打开手机 "ESP BLE Provisioning" App，扫描二维码配网
    // 长按 BOOT 按钮 3 秒可清除凭证，重新进入配网模式
    bool force_reset = ble_prov_check_reset();
    g_provisioning = true;

    ble_prov_start(
        // on_prov_start：进入 BLE 配网时显示二维码界面
        [](const char *qr_payload, const char *dev_name) {
            show_prov_qr_screen(qr_payload, dev_name);
            lv_timer_handler();
        },
        force_reset
    );

    // 等待 WiFi 连接，期间持续刷新 UI
    bool wifi_ok = ble_prov_wait_connected([]() {
        ui_set_status("连接 WiFi...");
        lv_timer_handler();
    });

    g_provisioning = false;

    // 配网/连接完成后切换到主界面（字体可能已加载完毕）
    if (g_main_scr) {
        lv_scr_load_anim(g_main_scr, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, true);
        Serial.println("[SETUP] 主界面切换完成");
    }

    if (wifi_ok) {
        ui_set_status("WiFi \xe5\xb7\xb2\xe8\xbf\x9e\xe6\x8e\xa5");  // "WiFi 已连接"
    } else {
        Serial.println("[WiFi] 连接失败，继续启动（无网络功能）");
        ui_set_status("WiFi \xe6\x9c\xaa\xe8\xbf\x9e\xe6\x8e\xa5");  // "WiFi 未连接"
    }

    Serial.printf("[MEM] 堆: %d, PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

    // 等待 BLE 配网流程彻底结束（PROV_END），确保 BLE 栈资源释放后再启动 I2S
    ble_prov_wait_done([]() { lv_timer_handler(); });

    // 启动音频任务（Core 1，高优先级）
    xTaskCreatePinnedToCore(audio_task, "audio_task", 12288, NULL, 5, NULL, 1);

    Serial.println("[SETUP] 初始化完成");
}

// ===== loop（Core 0，运行 LVGL）=====
void loop() {
    lv_timer_handler();
    delay(5);
}
