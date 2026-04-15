#pragma once
/**
 * 阿里云 NLS 语音合成 (TTS)
 * 流程：HTTP 下载 PCM → 写 WAV 到 SD → 用同一个 ESP_I2S 实例直接播放
 *
 * 不使用 ESP32-audioI2S 库，避免新旧 I2S 驱动冲突
 * WAV 播放：跳过 44 字节 header，直接把 PCM 数据写入 I2S（单声道→立体声扩展）
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SD_MMC.h>
#include "ESP_I2S.h"
#include "wifi_config.h"

#define TTS_URL      "https://nls-gateway-cn-shanghai.aliyuncs.com/stream/v1/tts"
#define TTS_WAV_PATH "/tts_tmp.wav"
#define WAV_HEADER_SIZE 44

extern I2SClass i2s;

// ── WAV header 写入 ────────────────────────────────────────────────
static void _write_wav_header(File &f, uint32_t sr, uint16_t ch,
                               uint16_t bits, uint32_t data_bytes) {
    uint32_t byte_rate   = sr * ch * bits / 8;
    uint16_t block_align = ch * bits / 8;
    uint32_t chunk_size  = 36 + data_bytes;
    uint32_t fmt_size    = 16;
    uint16_t fmt_type    = 1;

    f.write((uint8_t *)"RIFF", 4);
    f.write((uint8_t *)&chunk_size,  4);
    f.write((uint8_t *)"WAVE", 4);
    f.write((uint8_t *)"fmt ", 4);
    f.write((uint8_t *)&fmt_size,    4);
    f.write((uint8_t *)&fmt_type,    2);
    f.write((uint8_t *)&ch,          2);
    f.write((uint8_t *)&sr,          4);
    f.write((uint8_t *)&byte_rate,   4);
    f.write((uint8_t *)&block_align, 2);
    f.write((uint8_t *)&bits,        2);
    f.write((uint8_t *)"data", 4);
    f.write((uint8_t *)&data_bytes,  4);
}

// ── 从 SD 读取 WAV PCM 数据，单声道扩展为立体声写入 I2S ──────────
static void _play_wav_i2s(const char *path) {
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) { Serial.println("[TTS] 播放：文件打开失败"); return; }

    f.seek(WAV_HEADER_SIZE); // 跳过 WAV header，直接到 PCM 数据

    // 每次读 512 字节单声道，扩展为 1024 字节立体声
    const size_t MONO_CHUNK  = 512;
    const size_t STEREO_CHUNK = MONO_CHUNK * 2;
    uint8_t mono_buf[MONO_CHUNK];
    int16_t stereo_buf[MONO_CHUNK]; // MONO_CHUNK/2 个样本 × 2 声道

    Serial.println("[TTS] 播放中...");
    while (f.available()) {
        size_t n = f.read(mono_buf, MONO_CHUNK);
        if (n == 0) break;

        // 单声道 int16 → 立体声（左右相同）
        size_t samples = n / 2;
        int16_t *src = (int16_t *)mono_buf;
        for (size_t i = 0; i < samples; i++) {
            stereo_buf[i * 2]     = src[i];
            stereo_buf[i * 2 + 1] = src[i];
        }
        i2s.write((uint8_t *)stereo_buf, samples * 4);
    }
    f.close();
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
    Serial.printf("[TTS] PCM 大小: %d bytes\n", http.getSize());

    // 2. 流式下载 PCM → SD WAV
    SD_MMC.remove(TTS_WAV_PATH);
    File wav = SD_MMC.open(TTS_WAV_PATH, FILE_WRITE);
    if (!wav) {
        Serial.println("[TTS] SD 文件创建失败!");
        http.end();
        return false;
    }

    _write_wav_header(wav, TTS_SAMPLE_RATE, 1, 16, 0); // 占位 header

    // HTTPClient::writeToStream 自动处理 chunked 解码，只写纯 PCM 字节
    size_t total_pcm = http.writeToStream(&wav);
    http.end();
    Serial.printf("[TTS] 下载完成: %u bytes\n", (unsigned)total_pcm);

    // 回填真实 data size
    uint32_t data_bytes = (uint32_t)total_pcm;
    uint32_t riff_size  = 36 + data_bytes;
    wav.seek(4);  wav.write((uint8_t *)&riff_size,  4);
    wav.seek(40); wav.write((uint8_t *)&data_bytes, 4);
    wav.close();

    Serial.printf("[TTS] WAV 写入完成: %u bytes PCM\n", (unsigned)total_pcm);
    if (total_pcm == 0) return false;

    // 3. 直接用 ESP_I2S 播放（无需切换驱动）
    _play_wav_i2s(TTS_WAV_PATH);

    return true;
}
