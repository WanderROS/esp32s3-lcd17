#pragma once
/**
 * ESP-SR 语音识别配置
 *
 * 唤醒词模型名：传给 esp_srmodel_filter 的第三个参数（关键字模糊匹配）
 *   用 strings srmodels.bin | grep "^wn" 查看当前模型包可用的唤醒词
 *
 * 命令词语言：
 *   ESP_MN_ENGLISH  → 英文 (mn*_en)
 *   ESP_MN_CHINESE  → 中文 (mn*_cn)
 */

// 唤醒词关键字（当前模型包: wn9_xiaoaitongxue → "小爱同学"）
#define SR_WAKEWORD_NAME    "xiaoaitongxue"

// 命令词语言（当前模型包: mn5q8_en）
#define SR_MN_LANGUAGE      ESP_MN_ENGLISH
