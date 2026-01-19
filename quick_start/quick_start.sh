#!/bin/bash
# R1 Lite tmux 单窗口多 pane 启动脚本（工程级，带 adb 成功检测）

set -e

# ===================== 基本配置 =====================
SESSION_NAME="r1lite"

PASSWORD="1"

QUEST_DIR="$HOME/mobile_midea/quest"
PROTOTYPE_DIR="$HOME/mobile_midea/prototype"
QUICK_START_DIR="$HOME/mobile_midea/quick_start"

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
# 0: CAN
# 1: VR
# 2: ARM
# 3: DATA

tmux split-window -v -t "$SESSION_NAME:robot"     # 下半
tmux split-window -h -t "$SESSION_NAME:robot.1"   # 下左 / 下右
tmux split-window -h -t "$SESSION_NAME:robot.0"   # 上左 / 上右

tmux select-pane -t 0
tmux resize-pane -y 12

tmux select-pane -t 3
tmux resize-pane -y 10

# ===================== galaxea_robot_startup =====================
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

# ===================== VR pane =====================
tmux send-keys -t "$SESSION_NAME:robot.1" "
set +x
tmux select-pane -T 'VR…'
cd $QUEST_DIR || exit 1

# 配置 ADB 设备
bash bash/adb.sh

# 启动 VR ROS2 节点
source install/setup.bash
pixi run ros2 launch quest3s_vr vr_control.launch.py

" C-m

# ===================== ARM pane =====================
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
" C-m

# ===================== DATA_COLLECTOR pane =====================
tmux send-keys -t "$SESSION_NAME:robot.3" "
set +x
tmux select-pane -T 'DATA_COLLECTOR'
cd $PROTOTYPE_DIR || exit 1

# 启动 DATA_COLLECTOR 节点
source install/setup.bash
pixi run ros2 launch data_collector data_collector.launch.py

" C-m

# ===================== 进入 tmux =====================
tmux select-pane -t 0
tmux attach -t "$SESSION_NAME"
