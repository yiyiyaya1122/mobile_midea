#!/bin/bash
# 简单的CAN缓冲区检查脚本

CAN_IF=${1:-can0}

echo "=== Checking CAN buffer for $CAN_IF ==="
echo ""

# 检查接口是否存在
if [ ! -d "/sys/class/net/$CAN_IF" ]; then
    echo "Error: CAN interface $CAN_IF not found"
    exit 1
fi

# 显示统计信息
echo "Interface Statistics:"
echo "  RX packets: $(cat /sys/class/net/$CAN_IF/statistics/rx_packets 2>/dev/null || echo 'N/A')"
echo "  TX packets: $(cat /sys/class/net/$CAN_IF/statistics/tx_packets 2>/dev/null || echo 'N/A')"
echo "  RX dropped: $(cat /sys/class/net/$CAN_IF/statistics/rx_dropped 2>/dev/null || echo 'N/A')"
echo "  RX errors:  $(cat /sys/class/net/$CAN_IF/statistics/rx_errors 2>/dev/null || echo 'N/A')"
echo ""

# 使用ip命令显示详细信息
echo "Detailed Statistics:"
ip -s link show $CAN_IF 2>/dev/null | grep -E "RX|TX|dropped|errors" || echo "Unable to get detailed stats"
echo ""

# 检查是否有积压（使用ss命令）
echo "Socket Receive Queue:"
ss -i -m 2>/dev/null | grep -A 5 "$CAN_IF" || echo "No socket information available"
