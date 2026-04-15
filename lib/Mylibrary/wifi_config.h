#pragma once

// ===== WiFi 配置 =====
#define WIFI_SSID     "IOT_YF_301"
#define WIFI_PASSWORD "znjs301in2021."

// ===== 阿里云 ASR 配置 =====
// 控制台: https://nls-portal.console.aliyun.com/
#define ALIYUN_ASR_APPKEY  "ITUQNVR3Jrs3sZIU"
#define ALIYUN_ACCESS_TOKEN "35c197e5514740dcb1641fe4a07570f8"  // 从阿里云 NLS 获取

// ===== 阿里云百炼 API 配置 =====
// 控制台: https://bailian.console.aliyun.com → API-KEY 管理
// 兼容 OpenAI 接口: https://dashscope.aliyuncs.com/compatible-mode/v1
#define DASHSCOPE_API_KEY  "sk-24a87c6e507746389eb8b3a8321a7dce"
#define QWEN_MODEL         "qwen-turbo"  // 可选: qwen-plus, qwen-max, qwen-long

// ===== 阿里云 TTS 配置 =====
#define ALIYUN_TTS_APPKEY  "ITUQNVR3Jrs3sZIU"  // 通常与 ASR 相同
// TTS 发音人: zhixiaobai, zhixiaomei, zhichu, zhifeng 等
#define TTS_VOICE          "zhixiaobai"
#define TTS_SAMPLE_RATE    16000
