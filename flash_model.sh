#!/bin/bash
# Flash ESP-SR model to model partition

echo "Flashing srmodels.bin to model partition..."
echo "Note: Using address from partition table (0xd10000)"
~/.platformio/penv/bin/python ~/.platformio/packages/tool-esptoolpy/esptool.py \
  --chip esp32s3 \
  --port /dev/cu.usbmodem* \
  --baud 921600 \
  write_flash 0xd10000 srmodels.bin

echo "Done! Reset your ESP32-S3 to use ESP_SR wake word detection."
