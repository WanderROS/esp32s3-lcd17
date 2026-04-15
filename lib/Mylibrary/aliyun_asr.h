#pragma once
/**
 * 阿里云 NLS 语音识别 (ASR)
 * 使用 HTTP REST API (一句话识别) 而非 WebSocket，更适合嵌入式设备
 * API 文档: https://help.aliyun.com/document_detail/84435.html
 */

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "wifi_config.h"

// 阿里云 NLS 一句话识别 REST API
#define ASR_HOST "nls-gateway-cn-shanghai.aliyuncs.com"
#define ASR_URL  "https://nls-gateway-cn-shanghai.aliyuncs.com/stream/v1/asr"

/**
 * 将 PCM 音频数据发送到阿里云 ASR，返回识别文本
 * @param pcm_data  16kHz, 16bit, 单声道 PCM 数据
 * @param pcm_len   样本数量（非字节数）
 * @param result    输出识别结果
 * @return true 成功
 */
bool aliyun_asr_recognize(const int16_t *pcm_data, size_t pcm_len, String &result) {
    result = "";
    
    HTTPClient http;
    http.setTimeout(15000);
    
    String url = String(ASR_URL) +
                 "?appkey=" + ALIYUN_ASR_APPKEY +
                 "&format=pcm" +
                 "&sample_rate=" + String(TTS_SAMPLE_RATE) +
                 "&enable_punctuation_prediction=true" +
                 "&enable_inverse_text_normalization=true";
    
    http.begin(url);
    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("X-NLS-Token", ALIYUN_ACCESS_TOKEN);
    
    size_t byte_len = pcm_len * sizeof(int16_t);
    int httpCode = http.POST((uint8_t *)pcm_data, byte_len);
    
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[ASR] HTTP 错误: %d\n", httpCode);
        Serial.println(http.getString());
        http.end();
        return false;
    }
    
    String response = http.getString();
    http.end();
    
    Serial.printf("[ASR] 响应: %s\n", response.c_str());
    
    // 解析 JSON: {"status":20000000,"message":"SUCCESS","result":"识别文本"}
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, response);
    if (err) {
        Serial.printf("[ASR] JSON 解析失败: %s\n", err.c_str());
        return false;
    }
    
    int status = doc["status"] | 0;
    if (status != 20000000) {
        Serial.printf("[ASR] 识别失败, status=%d, msg=%s\n", status, doc["message"].as<const char*>());
        return false;
    }
    
    result = doc["result"].as<String>();
    Serial.printf("[ASR] 识别结果: %s\n", result.c_str());
    return true;
}
