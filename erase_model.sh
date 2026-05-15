#!/bin/bash
# Erase ESP-SR model partition (srmodels.bin)
# 注意：勿与 voice_data 分区混淆，两者现在是独立分区
# model 分区：0x910000, 大小 0x300000 (3MB)
# voice_data 分区：0xC10000, 大小 0x3E0000 (4MB)  ← 烧录 xiaole voice data

echo "Erasing model partition (0x910000, size 0x300000)..."
~/.platformio/penv/bin/python ~/.platformio/packages/tool-esptoolpy/esptool.py \
  --chip esp32s3 \
  --port /dev/cu.usbmodem* \
  --baud 921600 \
  erase_region 0x910000 0x300000

echo "Done! Model partition erased."
