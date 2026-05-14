#!/bin/bash
# 将 data/ 目录打包为 spiffs_fonts.bin 并烧录到 spiffs 分区 (0x610000, 2MB)

set -e

MKSPIFFS=~/.platformio/packages/tool-mkspiffs/mkspiffs_espressif32_arduino
PARTITION_OFFSET="0x710000"
PARTITION_SIZE="0x200000"   # 2MB
SPIFFS_BIN="spiffs_fonts.bin"

echo "=== 打包 data/ → $SPIFFS_BIN (${PARTITION_SIZE} bytes) ==="
$MKSPIFFS \
    -c data \
    -s $PARTITION_SIZE \
    -p 256 \
    -b 4096 \
    $SPIFFS_BIN

echo "=== 烧录 $SPIFFS_BIN → $PARTITION_OFFSET ==="
~/.platformio/penv/bin/python ~/.platformio/packages/tool-esptoolpy/esptool.py \
    --chip esp32s3 \
    --port /dev/cu.usbmodem* \
    --baud 921600 \
    write_flash "$PARTITION_OFFSET" "$SPIFFS_BIN"

echo "=== 完成！重启设备后字体文件即可从 Flash 读取 ==="
