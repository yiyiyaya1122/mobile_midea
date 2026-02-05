#!/bin/bash
# 激活 Galaxea CAN 接口配置脚本

echo ''
echo '🔧 配置 Galaxea CAN 接口...'

# 配置密码（可根据实际情况修改）
PASSWORD="${PASSWORD:-1}"

# 配置 can0
echo "$PASSWORD" | sudo -S ip link set down can0 2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set can0 type can \
    bitrate 1000000 sample-point 0.875 \
    dbitrate 5000000 fd on dsample-point 0.875 \
    2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set up can0 2>/dev/null || true
sleep 0.5

# 验证配置
echo ''
echo '📋 检查 CAN 接口状态:'
ip link show | grep can0 || echo "⚠ 未找到 can0 接口"

# 检查接口是否已启动
if ip link show can0 2>/dev/null | grep -q "UP"; then
    echo '✅ can0 已启动'
else
    echo '❌ can0 启动失败'
fi

echo '✅ Galaxea CAN 接口配置完成'
