#!/bin/bash
# R1 Lite 数据回放启动脚本（用于验证数据有效性）

set -e

# ===================== 基本配置 =====================
SESSION_NAME="r1lite_replay"

PASSWORD="1"

# 获取脚本所在目录（quick_start 目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 获取 mobile_midea 根目录（quick_start 的父目录）
MOBILE_MIDEA_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 基于根目录设置各模块路径
PROTOTYPE_DIR="$MOBILE_MIDEA_DIR/prototype"
QUICK_START_DIR="$MOBILE_MIDEA_DIR/quick_start"

# 默认参数
DEFAULT_HDF5_FILE=""
DEFAULT_CONFIG_PATH="$PROTOTYPE_DIR/src/inference/config/replay_config.yaml"
DEFAULT_LOOP="false"
DEFAULT_ARM_TYPE="yam"

# ===================== 参数解析 =====================
HDF5_FILE="${1:-$DEFAULT_HDF5_FILE}"
CONFIG_PATH="${2:-$DEFAULT_CONFIG_PATH}"
LOOP="${3:-$DEFAULT_LOOP}"
ARM_TYPE="${4:-$DEFAULT_ARM_TYPE}"

# 验证 arm_type
if [ "$ARM_TYPE" != "yam" ] && [ "$ARM_TYPE" != "arx" ]; then
    echo "❌ 错误: arm_type 必须是 'yam' 或 'arx'"
    exit 1
fi

# 检查必需参数
if [ -z "$HDF5_FILE" ] || [ ! -f "$HDF5_FILE" ]; then
    echo "❌ 错误: HDF5 文件未指定或不存在"
    echo ""
    echo "用法: $0 <hdf5_file> [config_path] [loop] [arm_type]"
    echo ""
    echo "参数说明:"
    echo "  hdf5_file   : HDF5 数据文件路径（必需）"
    echo "  config_path : 数据回放配置文件路径（可选，默认: $DEFAULT_CONFIG_PATH）"
    echo "  loop        : 是否循环播放（可选，默认: false）"
    echo "  arm_type    : 机械臂类型（可选，默认: yam，可选: yam 或 arx）"
    echo ""
    echo "示例:"
    echo "  $0 /path/to/collection_1234567890.hdf5"
    echo "  $0 /path/to/collection_1234567890.hdf5 \$CONFIG_PATH true yam"
    exit 1
fi

if [ ! -f "$CONFIG_PATH" ]; then
    echo "❌ 错误: 配置文件不存在: $CONFIG_PATH"
    exit 1
fi

echo "📋 回放配置:"
echo "  HDF5 文件: $HDF5_FILE"
echo "  配置文件: $CONFIG_PATH"
echo "  循环播放: $LOOP"
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
tmux new-session -d -s "$SESSION_NAME" -n replay

# ===================== Pane 布局 =====================
# 目标布局：3行2列
# 左列：CAN(0) | ARM(2) | Camera(4)
# 右列：DATA_REPLAY(1) | CONTROL(3) | Rviz2(5)
# 注意：pane编号由split顺序决定

# 创建左右两列
tmux split-window -h -t "$SESSION_NAME:replay"     # 水平分割：0(左), 1(右)

# 左列垂直分割成3行
tmux split-window -v -t "$SESSION_NAME:replay.0"   # 左列分割：0(左上), 2(左中)
tmux split-window -v -t "$SESSION_NAME:replay.2"   # 左列再分割：0(左上), 2(左中), 4(左下)

# 右列垂直分割成3行
tmux split-window -v -t "$SESSION_NAME:replay.1"   # 右列分割：1(右上), 3(右中)
tmux split-window -v -t "$SESSION_NAME:replay.3"    # 右列再分割：1(右上), 3(右中), 5(右下)

# 调整 pane 大小 - 使用 tiled 布局确保所有面板大小完全一致
tmux select-layout -t "$SESSION_NAME:replay" tiled

# ===================== galaxea_robot_startup pane =====================
tmux send-keys -t "$SESSION_NAME:replay.0" "
set +x
tmux select-pane -T 'CAN'
cd $PROTOTYPE_DIR || exit 1

# 配置 Galaxea CAN 接口
export PASSWORD='$PASSWORD'
bash $PROTOTYPE_DIR/CAN/Galaxea/activate_galaxea_can.sh

# 启动 galaxea_robot_startup.sh
bash $QUICK_START_DIR/galaxea_robot_startup.sh

exec bash
" C-m

# ===================== DATA_REPLAY pane =====================
# 注意：根据split顺序，DATA_REPLAY在pane 1（右列上）
tmux send-keys -t "$SESSION_NAME:replay.1" "
set +x
tmux select-pane -T 'DATA_REPLAY'
cd $PROTOTYPE_DIR || exit 1

# 启动数据回放节点
source install/setup.bash
pixi run ros2 launch inference data_replay.launch.py \
    hdf5_file:=$HDF5_FILE \
    config_path:=$CONFIG_PATH \
    loop:=$LOOP \
    arm_type:=$ARM_TYPE

exec bash
" C-m

# ===================== ARM pane =====================
# 注意：根据split顺序，ARM在pane 2（左列中）
tmux send-keys -t "$SESSION_NAME:replay.2" "
set +x
tmux select-pane -T 'ARM'
cd $PROTOTYPE_DIR || exit 1

# 配置 YAM CAN 接口
export PASSWORD='$PASSWORD'
bash $PROTOTYPE_DIR/CAN/YAM/activate_yam_can.sh

# 启动 ARM 控制节点
source install/setup.bash
pixi run ros2 launch yam_damiao_controller open_vr_double_arm.launch.py

exec bash
" C-m

# ===================== CONTROL pane =====================
# 注意：根据split顺序，CONTROL在pane 3（右列中）
tmux send-keys -t "$SESSION_NAME:replay.3" "
set +x
tmux select-pane -T 'CONTROL'
cd $QUICK_START_DIR || exit 1

bash control_replay.sh

exec bash
" C-m

# ===================== Camera pane =====================
# 注意：根据split顺序，Camera在pane 4（左列下，高度较小）
tmux send-keys -t "$SESSION_NAME:replay.4" "
set +x
tmux select-pane -T 'CAMERA'
cd $PROTOTYPE_DIR || exit 1

# 启动 UVC cameras
source install/setup.bash
./scripts/uvc_cameras.py

exec bash
" C-m

# ===================== Rviz2 pane =====================
# 注意：根据split顺序，Rviz2在pane 5（右列下，高度较小）
tmux send-keys -t "$SESSION_NAME:replay.5" "
set +x
tmux select-pane -T 'RVIZ2'
cd $PROTOTYPE_DIR || exit 1

# 启动 Rviz2
source install/setup.bash
rviz2

exec bash
" C-m

# ===================== 进入 tmux =====================
tmux select-pane -t 0
tmux attach -t "$SESSION_NAME"
