#!/bin/bash

# 定义目标文件
TARGET_FILE="adb.sh"

# 检查目标文件是否存在
if [ ! -f "$TARGET_FILE" ]; then
    echo "错误：文件 $TARGET_FILE 不存在"
    exit 1
fi

SERIAL_NUM=$(adb devices | grep -v "List" | awk '{print $1}' | head -n1)

# 验证是否成功获取序列号
if [ -z "$SERIAL_NUM" ]; then
    echo "错误：未检测到连接的Android设备"
    exit 1
fi

sudo sed -i "s|^SERIAL_NUM=.*|SERIAL_NUM=\"$SERIAL_NUM\"|" "$TARGET_FILE"

echo "替换结果："
grep "SERIAL_NUM" "$TARGET_FILE"

echo "已将设备序列号 $SERIAL_NUM 写入 $TARGET_FILE"
source $TARGET_FILE