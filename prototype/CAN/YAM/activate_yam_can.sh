#!/bin/bash
# 激活 YAM CAN 接口配置脚本

echo ''
echo '🔧 配置 YAM CAN 接口...'

# 配置密码（可根据实际情况修改）
PASSWORD="${PASSWORD:-1}"

# 配置 can_yam_l
echo "$PASSWORD" | sudo -S ip link set down can_yam_l 2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set can_yam_l type can bitrate 1000000 2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set can_yam_l txqueuelen 128 2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set up can_yam_l 2>/dev/null || true
sleep 0.5

# 配置 can_yam_r
echo "$PASSWORD" | sudo -S ip link set down can_yam_r 2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set can_yam_r type can bitrate 1000000 2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set can_yam_r txqueuelen 128 2>/dev/null || true
echo "$PASSWORD" | sudo -S ip link set up can_yam_r 2>/dev/null || true
sleep 0.5

# 验证配置
echo ''
echo '📋 检查 CAN 接口状态:'
ip link show | grep -E "can_yam_l|can_yam_r" || echo "⚠ 未找到 can_yam_l 或 can_yam_r 接口"

# 检查接口是否已启动
if ip link show can_yam_l 2>/dev/null | grep -q "UP"; then
    echo '✅ can_yam_l 已启动'
else
    echo '❌ can_yam_l 启动失败'
fi

if ip link show can_yam_r 2>/dev/null | grep -q "UP"; then
    echo '✅ can_yam_r 已启动'
else
    echo '❌ can_yam_r 启动失败'
fi

echo '✅ YAM CAN 接口配置完成'
