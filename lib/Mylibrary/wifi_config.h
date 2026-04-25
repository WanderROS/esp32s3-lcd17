#pragma once

// ===== WiFi 配置 =====
// WiFi 凭证已改为通过 BLE Provisioning 配网，不再硬编码
// 使用手机 App "ESP BLE Provisioning"（乐鑫官方）进行首次配网
// 凭证保存在 NVS Flash 中，重启后自动连接

// ===== 阿里云 ASR 配置 =====
// 控制台: https://nls-portal.console.aliyun.com/
#define ALIYUN_ASR_APPKEY  "ITUQNVR3Jrs3sZIU"
#define ALIYUN_ACCESS_TOKEN "169283f8d51048d1b07ae045d636af60"  // 从阿里云 NLS 获取

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
