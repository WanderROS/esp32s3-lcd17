#!/bin/bash
# Erase ESP-SR model partition

echo "Erasing model partition (0x710000, size 0x8E0000)..."
~/.platformio/penv/bin/python ~/.platformio/packages/tool-esptoolpy/esptool.py \
  --chip esp32s3 \
  --port /dev/cu.usbmodem* \
  --baud 921600 \
  erase_region 0x710000 0x8E0000

echo "Done! Model partition erased."
