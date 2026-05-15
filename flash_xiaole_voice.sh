#!/bin/bash
# 烧录 xiaole voice data 到 voice_data 分区 (0xC10000)
# ⚠️  注意：烧录到 voice_data 分区，不是 model 分区！
#     model 分区 (0x910000) 存放 ESP-SR 唤醒词模型 srmodels.bin
#     voice_data 分区 (0xC10000) 存放 TTS voice data

PORT="/dev/cu.usbmodem*"  # 根据实际情况修改
BAUD=921600
VOICE_DATA="esp_tts_voice_data_xiaoxin.dat"  # 也可以用 xiaoxin
ADDRESS="0xC10000"  # voice_data 分区地址

if [ ! -f "$VOICE_DATA" ]; then
    echo "错误: 未找到 $VOICE_DATA 文件"
    echo ""
    echo "可用的 voice data 文件："
    echo "  xiaoxin（女声，2.5MB）：已随 esp-skainet 附带"
    echo "    cp ~/Downloads/esp-skainet-master/examples/chinese_tts/esp_tts_voice_data_xiaoxin.dat ."
    echo "    mv esp_tts_voice_data_xiaoxin.dat $VOICE_DATA"
    exit 1
fi

echo "烧录 voice data 到 voice_data 分区 ($ADDRESS)..."
~/.platformio/penv/bin/python ~/.platformio/packages/tool-esptoolpy/esptool.py \
    --chip esp32s3 --port $PORT --baud $BAUD \
    write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB \
    $ADDRESS $VOICE_DATA

echo "烧录完成！"
