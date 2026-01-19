#!/bin/bash
# galaxea_robot_startup.sh 包装脚本 - 启动 Galaxea galaxea_robot_startup.sh

# 直接指定路径（不依赖环境变量）
STARTUP_SCRIPT_DIR="$HOME/galaxea/install/startup_config/share/startup_config/script"
STARTUP_SESSION="../sessions.d/ATCStandard/R1LITEBody.d"

echo ''
echo '🚀 启动 galaxea_robot_startup.sh'

# 切换到启动脚本目录
cd "$STARTUP_SCRIPT_DIR" || {
    echo "❌ startup_config 路径不存在: $STARTUP_SCRIPT_DIR"
    tmux select-pane -T 'CAN❌'
    exec bash
}

# 检查 galaxea_robot_startup.sh 是否存在
if [ ! -f "./galaxea_robot_startup.sh" ]; then
    echo "❌ galaxea_robot_startup.sh 不存在于: $STARTUP_SCRIPT_DIR"
    tmux select-pane -T 'CAN❌'
    exec bash
fi

# 执行 galaxea_robot_startup.sh
echo "📂 工作目录: $(pwd)"
echo "📋 会话配置: $STARTUP_SESSION"
if ./galaxea_robot_startup.sh boot "$STARTUP_SESSION"; then
    echo '✅ galaxea_robot_startup.sh 成功'
else
    echo '❌ galaxea_robot_startup.sh 失败'
    exec bash
fi