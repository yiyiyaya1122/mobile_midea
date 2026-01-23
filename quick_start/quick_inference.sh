#!/bin/bash
# R1 Lite 推理启动脚本

set -e

# ===================== 基本配置 =====================
SESSION_NAME="r1lite_inference"

# 获取脚本所在目录（quick_start 目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 获取 mobile_midea 根目录（quick_start 的父目录）
MOBILE_MIDEA_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 基于根目录设置各模块路径
PROTOTYPE_DIR="$MOBILE_MIDEA_DIR/prototype"

# 默认参数
DEFAULT_ARM_TYPE="yam"

# ===================== 参数解析 =====================
ARM_TYPE="${1:-$DEFAULT_ARM_TYPE}"

# 验证 arm_type
if [ "$ARM_TYPE" != "yam" ] && [ "$ARM_TYPE" != "arx" ]; then
    echo "❌ 错误: arm_type 必须是 'yam' 或 'arx'"
    echo ""
    echo "用法: $0 [arm_type]"
    echo ""
    echo "参数说明:"
    echo "  arm_type    : 机械臂类型（可选，默认: yam，可选: yam 或 arx）"
    echo ""
    echo "示例:"
    echo "  $0 yam"
    echo "  $0 arx"
    exit 1
fi

echo "📋 推理配置:"
echo "  机械臂类型: $ARM_TYPE"
echo ""

# ===================== 依赖检查 =====================
command -v tmux >/dev/null || { echo "❌ tmux 未安装"; exit 1; }
command -v pixi >/dev/null || { echo "❌ pixi 未安装"; exit 1; }

# ===================== 防止重复启动 =====================
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "⚠ tmux session [$SESSION_NAME] 已存在"
    echo "👉 tmux kill-session -t $SESSION_NAME"
    exit 1
fi

# ===================== 创建 session =====================
tmux new-session -d -s "$SESSION_NAME" -n inference

# ===================== Pane 布局 =====================
# 目标布局：1行1列（单个 pane）
# INFERENCE(0)

# ===================== INFERENCE pane =====================
tmux send-keys -t "$SESSION_NAME:inference.0" "
set +x
tmux select-pane -T 'INFERENCE'
cd $PROTOTYPE_DIR || exit 1

# 启动推理节点
source install/setup.bash
pixi run ros2 launch inference infer_${ARM_TYPE}.launch.py

exec bash
" C-m

# ===================== 进入 tmux =====================
tmux select-pane -t 0
tmux attach -t "$SESSION_NAME"
