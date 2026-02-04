#!/bin/bash
# smart_sync.sh - 智能同步最新日期的数据

# 配置
REMOTE_USER="chj"
REMOTE_HOST="192.168.124.13"
REMOTE_BASE_DIR="/media/chj/Extreme SSD/ltt"

# 自动获取最新日期目录
LATEST_DATE=$(ls ~/GalaxeaDataset/ | grep -E '^[0-9]{8}$' | sort | tail -1)

if [ -z "$LATEST_DATE" ]; then
    # 如果没有日期目录，使用今天日期
    LATEST_DATE=$(date +%Y%m%d)
    mkdir -p ~/GalaxeaDataset/$LATEST_DATE
fi

LOCAL_DIR="$HOME/GalaxeaDataset/$LATEST_DATE/"
REMOTE_DIR="$REMOTE_BASE_DIR/$LATEST_DATE/"

echo "同步日期: $LATEST_DATE"
echo "本地目录: $LOCAL_DIR"
echo "远程目录: $REMOTE_DIR"

# 同步命令（路径加引号，避免含空格时被拆成多个参数）
rsync -avz --progress \
    --exclude='*.tmp' \
    --exclude='.*' \
    "$LOCAL_DIR" \
    "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR"

# 记录日志
mkdir -p ~/GalaxeaDataset/logs
echo "$(date): 同步完成 $LATEST_DATE" >> ~/GalaxeaDataset/logs/sync.log
