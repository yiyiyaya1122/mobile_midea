#!/bin/bash
# R1 Lite tmux 单窗口多 pane 启动脚本（工程级，带 adb 成功检测）

set -e

# ===================== 基本配置 =====================
SESSION_NAME="r1lite_collect"

PASSWORD="1"

# 获取脚本所在目录（quick_start 目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 获取 mobile_midea 根目录（quick_start 的父目录）
MOBILE_MIDEA_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 基于根目录设置各模块路径
QUEST_DIR="$MOBILE_MIDEA_DIR/quest"
PROTOTYPE_DIR="$MOBILE_MIDEA_DIR/prototype"
QUICK_START_DIR="$MOBILE_MIDEA_DIR/quick_start"

CONFIG_CAN=true
START_VR=true
START_ARM=true
START_DATA=true

# ===================== 依赖检查 =====================
command -v tmux >/dev/null || { echo "❌ tmux 未安装"; exit 1; }
command -v pixi >/dev/null || { echo "❌ pixi 未安装"; exit 1; }
command -v adb >/dev/null || { echo "❌ adb 未安装"; exit 1; }

# ===================== 防止重复启动 =====================
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "⚠ tmux session [$SESSION_NAME] 已存在"
    echo "👉 tmux kill-session -t $SESSION_NAME"
    exit 1
fi

# ===================== 创建 session =====================
tmux new-session -d -s "$SESSION_NAME" -n robot

# ===================== Pane 布局 =====================
# 目标布局：3行2列
# 左列：CAN(0) | ARM(2) | Camera(4)
# 右列：DATA(1) | VR(3) | Rviz2(5)
# 注意：pane编号由split顺序决定

# 创建左右两列
tmux split-window -h -t "$SESSION_NAME:robot"     # 水平分割：0(左), 1(右)

# 左列垂直分割成3行
tmux split-window -v -t "$SESSION_NAME:robot.0"   # 左列分割：0(左上), 2(左中)
tmux split-window -v -t "$SESSION_NAME:robot.2"   # 左列再分割：0(左上), 2(左中), 4(左下)

# 右列垂直分割成3行
tmux split-window -v -t "$SESSION_NAME:robot.1"   # 右列分割：1(右上), 3(右中)
tmux split-window -v -t "$SESSION_NAME:robot.3"    # 右列再分割：1(右上), 3(右中), 5(右下)

# 调整 pane 大小 - 使用 tiled 布局确保所有面板大小完全一致
tmux select-layout -t "$SESSION_NAME:robot" tiled

# ===================== galaxea_robot_startup pane=====================
tmux send-keys -t "$SESSION_NAME:robot.0" "
set +x
tmux select-pane -T 'CAN'

# 配置 Galaxea CAN 接口
export PASSWORD='$PASSWORD'
bash $PROTOTYPE_DIR/CAN/Galaxea/activate_galaxea_can.sh

# 启动 galaxea_robot_startup.sh
bash $QUICK_START_DIR/galaxea_robot_startup.sh

exec bash
" C-m

# ===================== DATA_COLLECTOR pane =====================
# 注意：根据split顺序，DATA在pane 1（右列上）
tmux send-keys -t "$SESSION_NAME:robot.1" "
set +x
tmux select-pane -T 'DATA_COLLECTOR'
cd $PROTOTYPE_DIR || exit 1

# 启动 DATA_COLLECTOR 节点
source install/setup.bash
pixi run ros2 launch data_collector data_collector.launch.py

exec bash
" C-m

# ===================== ARM pane =====================
# 注意：根据split顺序，ARM在pane 2（左列中）
tmux send-keys -t "$SESSION_NAME:robot.2" "
set +x
tmux select-pane -T 'ARM…'
cd $PROTOTYPE_DIR || exit 1

# 配置 YAM CAN 接口
export PASSWORD='$PASSWORD'
bash $PROTOTYPE_DIR/CAN/YAM/activate_yam_can.sh

# 启动 ARM 控制节点
source install/setup.bash
pixi run ros2 launch yam_damiao_controller open_vr_double_arm.launch.py

exec bash
" C-m

# ===================== VR pane =====================
# 注意：根据split顺序，VR在pane 3（右列中）
tmux send-keys -t "$SESSION_NAME:robot.3" "
set +x
tmux select-pane -T 'VR…'
cd $QUEST_DIR || exit 1

# 配置 ADB 设备，如果失败则退出
bash bash/adb.sh || exit 1

# 启动 VR ROS2 节点
source install/setup.bash
pixi run ros2 launch quest3s_vr vr_control.launch.py

exec bash
" C-m

# ===================== Camera pane =====================
# 注意：根据split顺序，Camera在pane 4（左列下，高度较小）
tmux send-keys -t "$SESSION_NAME:robot.4" "
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
tmux send-keys -t "$SESSION_NAME:robot.5" "
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
