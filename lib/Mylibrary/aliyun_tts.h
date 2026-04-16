#pragma once
/**
 * 阿里云 NLS 语音合成 (TTS)
 * 流程：HTTP 下载 PCM（由 writeToStream 自动处理 chunked 解码）
 *       → 存入 PSRAM 缓冲区 → 直接从内存播放
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "ESP_I2S.h"
#include "wifi_config.h"

#define TTS_URL "https://nls-gateway-cn-shanghai.aliyuncs.com/stream/v1/tts"

extern I2SClass i2s;

// ── PSRAM 内存 sink：实现 Print 接口，供 writeToStream 写入 ──────
class PsramStream : public Stream {
public:
    // Stream 读接口（只写不读，空实现）
    int  available() override { return 0; }
    int  read()      override { return -1; }
    int  peek()      override { return -1; }
public:
    uint8_t *buf  = nullptr;
    size_t   size = 0;   // 已分配容量
    size_t   len  = 0;   // 已写入字节数

    bool init(size_t initial) {
        buf = (uint8_t *)heap_caps_malloc(initial, MALLOC_CAP_SPIRAM);
        if (!buf) return false;
        size = initial;
        len  = 0;
        return true;
    }

    void release() {
        if (buf) { heap_caps_free(buf); buf = nullptr; }
        size = len = 0;
    }

    size_t write(uint8_t c) override {
        return write(&c, 1);
    }

    size_t write(const uint8_t *data, size_t n) override {
        if (len + n > size) {
            // 扩容：每次增加 64KB
            size_t new_size = size + max(n, (size_t)(64 * 1024));
            uint8_t *tmp = (uint8_t *)heap_caps_realloc(buf, new_size, MALLOC_CAP_SPIRAM);
            if (!tmp) {
                Serial.println("[TTS] PSRAM 扩容失败，截断");
                n = size - len; // 写满为止
                if (n == 0) return 0;
            } else {
                buf  = tmp;
                size = new_size;
            }
        }
        memcpy(buf + len, data, n);
        len += n;
        return n;
    }
};

// ── 从 PSRAM 缓冲区播放 PCM，单声道扩展为立体声写入 I2S ──────────
static void _play_pcm_psram(const uint8_t *pcm_buf, size_t pcm_len) {
    if (!pcm_buf || pcm_len == 0) return;

    const size_t MONO_CHUNK = 512;
    int16_t stereo_buf[MONO_CHUNK]; // MONO_CHUNK/2 个样本 × 2 声道

    Serial.println("[TTS] 播放中...");
    size_t offset = 0;
    while (offset < pcm_len) {
        size_t n = min(MONO_CHUNK, pcm_len - offset);
        const int16_t *src = (const int16_t *)(pcm_buf + offset);
        size_t samples = n / 2;

        for (size_t i = 0; i < samples; i++) {
            stereo_buf[i * 2]     = src[i];
            stereo_buf[i * 2 + 1] = src[i];
        }
        i2s.write((uint8_t *)stereo_buf, samples * 4);
        offset += n;
    }
    Serial.println("[TTS] 播放完成");
}

// ── 主函数 ────────────────────────────────────────────────────────
bool aliyun_tts_speak(const String &text) {
    if (text.isEmpty()) return false;

    // 1. 构建 JSON 请求
    DynamicJsonDocument doc(512);
    doc["appkey"]      = ALIYUN_TTS_APPKEY;
    doc["token"]       = ALIYUN_ACCESS_TOKEN;
    doc["text"]        = text;
    doc["format"]      = "pcm";
    doc["sample_rate"] = TTS_SAMPLE_RATE;
    doc["voice"]       = TTS_VOICE;
    doc["volume"]      = 50;
    doc["speech_rate"] = 0;
    doc["pitch_rate"]  = 0;
    String body;
    serializeJson(doc, body);

    Serial.printf("[TTS] 合成: %s\n", text.c_str());

    HTTPClient http;
    http.setTimeout(20000);
    http.begin(TTS_URL);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(body);

    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[TTS] HTTP 错误: %d %s\n", httpCode, http.getString().c_str());
        http.end();
        return false;
    }

    int content_len = http.getSize();
    Serial.printf("[TTS] PCM 大小: %d bytes\n", content_len);

    // 2. 初始化 PSRAM sink（content_len 未知时从 64KB 起步）
    PsramStream sink;
    size_t initial = (content_len > 0) ? (size_t)content_len : 64 * 1024;
    if (!sink.init(initial)) {
        Serial.printf("[TTS] PSRAM 分配失败，可用最大块: %u bytes\n",
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        http.end();
        return false;
    }

    // 3. writeToStream 自动处理 chunked 解码，只写纯 PCM 字节
    size_t total = http.writeToStream(&sink);
    http.end();

    Serial.printf("[TTS] 下载完成: %u bytes\n", (unsigned)total);
    if (total == 0) {
        sink.release();
        return false;
    }

    // 4. 直接从 PSRAM 播放
    _play_pcm_psram(sink.buf, sink.len);

    sink.release();
    return true;
}
