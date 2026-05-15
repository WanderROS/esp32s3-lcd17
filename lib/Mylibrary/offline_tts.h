#pragma once
/**
 * offline_tts.h — 基于 ESP-TTS 的离线中文语音合成
 *
 * 使用 xiaoxin 音色（完整中文词表）+ 外部 Flash 分区（voice_data）存放 PCM 数据。
 * 支持直接输入任意中文汉字（通过 esp_tts_parse_chinese），无需手动拼音。
 *
 * 准备工作：
 * 1. 分区表（esp_sr_16_large.csv）包含独立的 voice_data 分区（0xC10000，4MB，fat 类型）
 *    model 分区（0x910000，3MB）保留给 ESP-SR 唤醒词模型 srmodels.bin，两者互不干扰
 * 2. 将 esp_tts_voice_data_xiaoxin.dat 烧录到 voice_data 分区：
 *    ./flash_xiaole_voice.sh
 *    （文件来自 esp-skainet/examples/chinese_tts/esp_tts_voice_data_xiaoxin.dat）
 *
 * 用法：
 *   offline_tts_init();                    // 初始化（一次）
 *   offline_tts_speak("你好，我是小爱同学");  // 直接传汉字，自动合成播放
 */

#include <Arduino.h>
#include "esp_tts.h"
#include "esp_tts_voice_template.h"
#include "esp_partition.h"
#include "ESP_I2S.h"

extern I2SClass i2s;

// ── 全局 TTS 句柄（单例） ─────────────────────────────────────
static esp_tts_handle_t s_tts_handle = nullptr;
static esp_tts_voice_t *s_voice = nullptr;

/**
 * 初始化 ESP-TTS（xiaoxin 音色 + Flash voice_data 分区映射），必须在 I2S 初始化后调用。
 * voice_data 分区（fat，0xC10000）存放 xiaoxin PCM，与 model 分区（ESP-SR）相互独立。
 * @return true  成功
 */
static bool offline_tts_init() {
    if (s_tts_handle) return true;  // 已初始化

    // 1. 查找 voice_data 分区（fat 类型，0xC10000，与 model/ESP-SR 分区完全独立）
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "voice_data");
    if (!part) {
        Serial.println("[TTS] 错误：未找到 voice_data 分区");
        Serial.println("[TTS] 请检查分区表并执行 ./flash_xiaole_voice.sh 烧录 voice data");
        return false;
    }
    Serial.printf("[TTS] 找到分区: %s, 地址 0x%x, 大小 %d bytes\n",
                  part->label, part->address, part->size);

    // 2. 内存映射 PCM 数据到虚拟地址空间
    const void *voicedata = nullptr;
    esp_partition_mmap_handle_t mmap;
    esp_err_t err = esp_partition_mmap(part, 0, part->size,
                                       ESP_PARTITION_MMAP_DATA, &voicedata, &mmap);
    if (err != ESP_OK) {
        Serial.printf("[TTS] 内存映射失败: %s\n", esp_err_to_name(err));
        return false;
    }

    // 3. 用 template + 外部 PCM 数据初始化（xiaoxin/xiaole dat 文件通用此方式）
    s_voice = esp_tts_voice_set_init(&esp_tts_voice_template, (int16_t *)voicedata);
    s_tts_handle = esp_tts_create(s_voice);
    if (!s_tts_handle) {
        Serial.println("[TTS] esp_tts_create 失败");
        return false;
    }
    Serial.println("[TTS] ESP-TTS 初始化成功 (xiaoxin)");
    return true;
}

/**
 * 通过中文汉字合成语音并通过 I2S 播放（阻塞直到播完）。
 *
 * xiaoxin 音色支持完整中文常用字（约 3500 字），直接传入汉字即可，无需拼音。
 * @param text    中文文本，如 "你好世界"
 * @param speed   语速 0~5，默认 3
 */
static void offline_tts_speak(const char *text, unsigned int speed = 3) {
    if (!s_tts_handle) {
        Serial.println("[TTS] 未初始化，跳过");
        return;
    }
    if (!text || text[0] == '\0') return;

    Serial.printf("[TTS] 合成文本: %s\n", text);

    // 使用 esp_tts_parse_chinese 解析汉字（自动转拼音）
    char buf[256];
    strncpy(buf, text, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (!esp_tts_parse_chinese(s_tts_handle, buf)) {
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
