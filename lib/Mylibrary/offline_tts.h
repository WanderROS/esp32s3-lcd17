#pragma once
/**
 * offline_tts.h — 基于 ESP-TTS 的离线中文语音合成
 *
 * 使用乐鑫内置 xiaoxin 音色（PCM 数据完全内嵌于 libvoice_set_xiaole.a，无需外部 Flash 分区）。
 * xiaole 音色的 PCM 数据依赖外部 voice data 分区，不适合直接使用。
 * ESP-TTS 只支持中文拼音/常用汉字，英文需转拼音后传入。
 *
 * 用法：
 *   offline_tts_init();                    // 初始化（一次）
 *   offline_tts_speak("你好，我是小爱同学");  // 合成并播放
 */

#include <Arduino.h>
#include "esp_tts.h"
#include "ESP_I2S.h"

extern I2SClass i2s;

// xiaoxin 音色：PCM 完全内嵌在 libvoice_set_xiaole.a（3.5MB），无需外部 Flash 分区
// 通过 extern "C" 声明强制链接器拉入 esp_tts_voice_xiaoxin.c.obj
extern "C" const esp_tts_voice_t esp_tts_voice_xiaoxin;

// ── 全局 TTS 句柄（单例） ─────────────────────────────────────
static esp_tts_handle_t s_tts_handle = nullptr;

/**
 * 初始化 ESP-TTS，必须在 I2S 初始化完成后调用。
 * xiaoxin 音色：PCM 数据内嵌在静态库中（xiaoxin_syll_data），无需外部 Flash 分区。
 * @return true  成功
 */
static bool offline_tts_init() {
    if (s_tts_handle) return true;  // 已初始化

    // 使用 xiaoxin 音色：其 syll_data 完整编译进 libvoice_set_xiaole.a（约 4MB）
    // xiaole 音色 PCM 依赖外部 model 分区，不能直接用
    s_tts_handle = esp_tts_create(const_cast<esp_tts_voice_t *>(&esp_tts_voice_xiaoxin));
    if (!s_tts_handle) {
        Serial.println("[TTS] esp_tts_create 失败");
        return false;
    }
    Serial.println("[TTS] ESP-TTS 初始化成功 (xiaoxin)");
    return true;
}

/**
 * 通过拼音合成语音并通过 I2S 播放（阻塞直到播完）。
 *
 * xiaoxin 音色词表为支付/播报场景，支持声调拼音输入。
 * 拼音格式：音节+声调数字，【逗号】分隔（反汇编确认分隔符为 0x2C=','）
 *   正确示例: "nin2,hao3"   （逗号，不是空格！）
 * 支持的拼音可通过 strings libvoice_set_xiaole.a | grep "^[a-z]*[1-4]$" 查看。
 *
 * @param pinyin  拼音字符串，逗号分隔，如 "nin2,hao3"
 * @param speed   语速 0~5，默认 3
 */
static void offline_tts_speak(const char *pinyin, unsigned int speed = 3) {
    if (!s_tts_handle) {
        Serial.println("[TTS] 未初始化，跳过");
        return;
    }
    if (!pinyin || pinyin[0] == '\0') return;

    Serial.printf("[TTS] 合成拼音: %s\n", pinyin);

    // 注意：esp_tts_parse_pinyin 以逗号','为分隔符，不是空格
    // 需要一个可写的 char[] 副本（函数签名是 char* 非 const char*）
    char buf[64];
    strncpy(buf, pinyin, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (!esp_tts_parse_pinyin(s_tts_handle, buf)) {
        Serial.println("[TTS] 解析失败");
        return;
    }

    // 流式输出 PCM，单声道扩展为立体声写入 I2S
    const size_t MONO_CHUNK = 512;   // 每次处理的单声道样本数上限
    int16_t stereo_buf[MONO_CHUNK * 2];
    int chunk_len = 0;
    int chunk_count = 0;  // 用于定期让出 CPU

    while (true) {
        short *pcm = esp_tts_stream_play(s_tts_handle, &chunk_len, speed);
        if (chunk_len == 0 || pcm == nullptr) break;  // 播放完毕

        // 分批写入，避免 stereo_buf 溢出
        int offset = 0;
        while (offset < chunk_len) {
            int batch = min((int)(MONO_CHUNK), chunk_len - offset);
            for (int i = 0; i < batch; i++) {
                stereo_buf[i * 2]     = pcm[offset + i];  // 左声道
                stereo_buf[i * 2 + 1] = pcm[offset + i];  // 右声道
            }
            i2s.write((uint8_t *)stereo_buf, batch * 4);
            offset += batch;
        }

        // 每 20 个 chunk 让出一次 CPU，防止 WDT 超时
        if (++chunk_count % 20 == 0) {
            vTaskDelay(1);
        }
    }

    esp_tts_stream_reset(s_tts_handle);
    Serial.println("[TTS] 播放完成");
}

/**
 * 释放 ESP-TTS 资源（一般不需要调用）。
 */
static void offline_tts_deinit() {
    if (s_tts_handle) {
        esp_tts_destroy(s_tts_handle);
        s_tts_handle = nullptr;
    }
}
