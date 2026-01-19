#!/bin/bash
# ADB 设备配置脚本 - 自动检测设备序列号并配置端口转发

echo ''
echo '🔧 配置 ADB 设备...'

# 重启 adb 服务器以确保连接正常
adb kill-server
sleep 1

# 自动检测连接的 Android 设备序列号
SERIAL_NUM=$(adb devices | grep -v "List" | grep "device" | awk '{print $1}' | head -n1)

# 验证是否成功获取序列号
if [ -z "$SERIAL_NUM" ]; then
    echo "❌ 错误：未检测到连接的 Android 设备"
    echo "   请确保设备已通过 USB 连接并已启用 USB 调试"
    exit 1
fi

echo "✅ 检测到 Android 设备: $SERIAL_NUM"

# 显示设备列表
adb devices

# 设置端口反向代理（用于 VR 通信）
echo "📡 配置端口反向代理..."
if adb reverse tcp:8012 tcp:8012; then
    echo "✅ 端口反向代理配置成功 (tcp:8012 -> tcp:8012)"
else
    echo "⚠ 端口反向代理配置失败，但继续执行"
fi

# 如果需要通过 TCP 连接，可以使用以下命令（取消注释）
# adb tcpip 5555
# adb connect 182.168.0.183:5555

# 如果需要指定设备执行命令，可以使用：
# adb -s $SERIAL_NUM <command>

echo "✅ ADB 设备配置完成"
