#!/bin/bash
# 测试脚本：检查 xiaoxin voice data 文件是否可用

VOICE_DATA="esp_tts_voice_data_xiaoxin.dat"

echo "=== 离线 TTS Voice Data 检查 (xiaoxin) ==="
echo ""

# 检查文件是否存在
if [ ! -f "$VOICE_DATA" ]; then
    echo "❌ 未找到 $VOICE_DATA"
    echo ""
    echo "请从 esp-skainet 示例目录复制："
    echo "  cp ~/Downloads/esp-skainet-master/examples/chinese_tts/esp_tts_voice_data_xiaoxin.dat ."
    echo ""
    echo "将文件放到项目根目录后重新运行此脚本。"
    exit 1
fi

# 检查文件大小
SIZE=$(wc -c < "$VOICE_DATA")
SIZE_MB=$((SIZE / 1024 / 1024))

echo "✅ 找到文件: $VOICE_DATA"
echo "📊 文件大小: ${SIZE_MB}MB ($SIZE bytes)"
echo ""

# 合理性检查
if [ $SIZE_MB -lt 2 ] || [ $SIZE_MB -gt 10 ]; then
    echo "⚠️  警告：文件大小异常（预期约 2.5MB）"
fi

echo "✅ 准备就绪，运行以下命令烧录："
echo "   ./flash_xiaole_voice.sh"
