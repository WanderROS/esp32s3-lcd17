# 离线 TTS (xiaole 音色) 配置指南

## 概述
本项目已从 `xiaoxin` (支付播报专用) 切换到 `xiaole` (完整中文支持)。

## 配置步骤

### 1. 获取 voice data (3种方法)

**方法 1：使用官方示例自带的 xiaoxin（最快）**
```bash
cp ~/Downloads/esp-skainet-master/examples/chinese_tts/esp_tts_voice_data_xiaoxin.dat \
   ~/Documents/PlatformIO/esp32s3-lcd17/
```

**方法 2：克隆 ESP-SR 获取 xiaole**
```bash
git clone --depth 1 https://github.com/espressif/esp-sr.git
find esp-sr -name "*xiaole*.dat" -o -name "*voice_data*"
```

**方法 3：检查 PlatformIO libs**
```bash
find ~/.platformio/packages/framework-arduinoespressif32-libs -name "*voice*"
```

### 2. 烧录到 Flash
```bash
cd ~/Documents/PlatformIO/esp32s3-lcd17
chmod +x flash_xiaole_voice.sh
./flash_xiaole_voice.sh
```
