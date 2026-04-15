#pragma once
/**
 * 阿里云百炼 通义千问 API
 * 使用 OpenAI 兼容接口（百炼平台统一入口）
 * 文档: https://help.aliyun.com/zh/model-studio/developer-reference/use-qwen-by-calling-api
 * Endpoint: POST https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "wifi_config.h"

#define QWEN_API_URL "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"

// 多轮对话历史（保留最近 N 轮）
#define MAX_HISTORY_TURNS 5

struct ChatMessage {
    String role;
    String content;
};

static ChatMessage chat_history[MAX_HISTORY_TURNS * 2];
static int history_count = 0;

void qwen_clear_history() {
    history_count = 0;
}

/**
 * 调用百炼通义千问（OpenAI 兼容格式）
 * @param user_input 用户输入
 * @param reply      输出 AI 回复
 * @return true 成功
 */
bool qwen_chat(const String &user_input, String &reply) {
    reply = "";

    // 构建请求体（OpenAI chat/completions 格式）
    DynamicJsonDocument req_doc(4096);
    req_doc["model"] = QWEN_MODEL;
    req_doc["max_tokens"] = 512;

    JsonArray messages = req_doc.createNestedArray("messages");

    // 系统提示
    JsonObject sys_msg = messages.createNestedObject();
    sys_msg["role"]    = "system";
    sys_msg["content"] = "你是一个智能语音助手，回答简洁友好，适合语音播报，不使用 Markdown 格式，不使用符号列表。";

    // 历史对话
    for (int i = 0; i < history_count; i++) {
        JsonObject msg    = messages.createNestedObject();
        msg["role"]       = chat_history[i].role;
        msg["content"]    = chat_history[i].content;
    }

    // 当前用户输入
    JsonObject user_msg    = messages.createNestedObject();
    user_msg["role"]       = "user";
    user_msg["content"]    = user_input;

    String req_body;
    serializeJson(req_doc, req_body);

    HTTPClient http;
    http.setTimeout(30000);
    http.begin(QWEN_API_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + DASHSCOPE_API_KEY);

    Serial.printf("[Qwen] 发送请求，用户: %s\n", user_input.c_str());
    int httpCode = http.POST(req_body);

    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[Qwen] HTTP 错误: %d\n", httpCode);
        Serial.println(http.getString());
        http.end();
        return false;
    }

    String response = http.getString();
    http.end();

    // 解析 OpenAI 格式响应
    // {"choices":[{"message":{"role":"assistant","content":"..."}}],...}
    DynamicJsonDocument resp_doc(4096);
    DeserializationError err = deserializeJson(resp_doc, response);
    if (err) {
        Serial.printf("[Qwen] JSON 解析失败: %s\n", err.c_str());
        return false;
    }

    if (resp_doc.containsKey("error")) {
        Serial.printf("[Qwen] API 错误: %s\n",
            resp_doc["error"]["message"].as<const char *>());
        return false;
    }

    reply = resp_doc["choices"][0]["message"]["content"].as<String>();
    reply.trim();
    Serial.printf("[Qwen] 回复: %s\n", reply.c_str());

    // 更新对话历史（超出上限时滚动丢弃最旧一轮）
    if (history_count >= MAX_HISTORY_TURNS * 2) {
        for (int i = 0; i < history_count - 2; i++) {
            chat_history[i] = chat_history[i + 2];
        }
        history_count -= 2;
    }
    chat_history[history_count++] = {"user",      user_input};
    chat_history[history_count++] = {"assistant", reply};

    return true;
}
