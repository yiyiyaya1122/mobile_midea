# Quick Start 快速上手指南

本指南用于在机器人端（R1 Lite）通过 SSH 一键启动**采集**、**回放**、**推理**等流程。所有脚本均在 tmux 中运行，便于多窗格管理。

---

## 前置条件

- **环境**：机器人端已安装 `tmux`、`pixi`；采集/VR 流程需安装 `adb`。
- **机器人启动**：采集与回放会先配置 CAN 并启动 `galaxea_robot_startup.sh`，请确保 `~/galaxea/install/startup_config/...` 下存在对应启动脚本。
- **网络**：本机可 SSH 到机器人（示例 IP：`192.168.124.2`）。

---

## 一、SSH 连接

1. 打开终端：`Ctrl + Alt + T`
2. 连接机器人：
   ```bash
   ssh r1lite@192.168.124.2
   ```
   密码：`1`
3. 进入工程目录：
   ```bash
   cd ~/mobile_midea/quick_start
   ```

---

## 二、采集（数据收集）

启动后会在 tmux 中打开 **3×2** 窗格，自动完成：Galaxea CAN 配置与底盘启动、YAM CAN 与双臂启动、数据采集节点、VR 节点、UVC 相机、Rviz2。

### 启动

```bash
cd ~/mobile_midea/quick_start
bash quick_collect.sh
```

### 窗格布局（从左到右、上到下）

| 窗格 | 标题       | 内容说明 |
|------|------------|----------|
| 0    | CAN        | Galaxea CAN 配置 + `galaxea_robot_startup.sh` |
| 1    | DATA_COLLECTOR | 数据采集节点 `data_collector` |
| 2    | ARM…       | YAM CAN 配置 + 双臂控制器 `open_vr_double_arm` |
| 3    | VR…        | Quest VR 节点（需 ADB 已配置） |
| 4    | CAMERA     | UVC 相机 `uvc_cameras.py` |
| 5    | RVIZ2      | Rviz2 |

### 停止采集

在任意终端执行（或在 tmux 外新开终端 SSH 后执行）：

```bash
tmux kill-session -t r1lite_collect
```

---

## 三、回放（数据回放）

用于验证已采集的 HDF5 数据，会启动：Galaxea CAN + 底盘、数据回放节点、YAM 双臂、回放控制说明、UVC 相机、Rviz2。

### 启动

```bash
cd ~/mobile_midea/quick_start
bash quick_replay.sh <hdf5文件路径> [config_path] [loop] [arm_type]
```

- **hdf5_file**（必填）：HDF5 数据文件路径。
- **config_path**（可选）：回放配置，默认 `prototype/src/inference/config/replay_config.yaml`。
- **loop**（可选）：是否循环，默认 `false`。
- **arm_type**（可选）：`yam` 或 `arx`，默认 `yam`。

示例：

```bash
bash quick_replay.sh /path/to/collection_1234567890.hdf5
bash quick_replay.sh /path/to/data.hdf5 "" true yam
```

### 窗格布局

| 窗格 | 标题        | 内容说明 |
|------|-------------|----------|
| 0    | CAN         | Galaxea CAN + `galaxea_robot_startup.sh` |
| 1    | DATA_REPLAY | 数据回放节点 |
| 2    | ARM         | YAM CAN + 双臂控制器 |
| 3    | CONTROL     | 回放控制说明（见下） |
| 4    | CAMERA      | UVC 相机 |
| 5    | RVIZ2       | Rviz2 |

### 回放控制（在 CONTROL 窗格或任意终端）

- **开始回放**：
  ```bash
  ros2 service call /start_replay std_srvs/srv/Trigger
  ```
- **停止回放**：
  ```bash
  ros2 service call /stop_replay std_srvs/srv/Trigger
  ```

### 停止回放会话

```bash
tmux kill-session -t r1lite_replay
```

---

## 四、推理（VLA 推理）

仅启动推理节点，用于 YAM 或 ARX 机械臂的推理控制。

### 启动

```bash
cd ~/mobile_midea/quick_start
bash quick_inference.sh [arm_type]
```

- **arm_type**（可选）：`yam` 或 `arx`，默认 `yam`。

示例：

```bash
bash quick_inference.sh
bash quick_inference.sh yam
bash quick_inference.sh arx
```

### 停止

```bash
tmux kill-session -t r1lite_inference
```

---

## 五、tmux 基本操作

| 操作       | 按键 |
|------------|------|
| 前缀键     | `Ctrl + B` |
| 脱离当前会话（会话继续在后台） | 前缀键 + `D` |
| 在窗格间切换 | 前缀键 + 方向键 |
| 重新连接已有会话 | `tmux attach -t <session_name>` |

### 常用 session 名称

- 采集：`r1lite_collect`
- 回放：`r1lite_replay`
- 推理：`r1lite_inference`

重新连接示例：

```bash
tmux attach -t r1lite_collect
```

---

## 六、脚本与依赖速查

| 脚本 | 作用 | 依赖 |
|------|------|------|
| `quick_collect.sh` | 采集（CAN + 底盘 + 双臂 + 采集 + VR + 相机 + Rviz） | tmux, pixi, adb |
| `quick_replay.sh` | 回放（CAN + 底盘 + 双臂 + 回放 + 控制说明 + 相机 + Rviz） | tmux, pixi；需提供 HDF5 路径 |
| `quick_inference.sh` | 推理（仅推理节点） | tmux, pixi |
| `galaxea_robot_startup.sh` | 底盘/机器人启动（由采集/回放自动调用） | 需已安装 galaxea startup_config |
| `control_replay.sh` | 打印回放控制命令（由 quick_replay 在 CONTROL 窗格中运行） | 无 |

---

## 七、常见问题

- **“tmux session 已存在”**：先结束旧会话再启动，例如 `tmux kill-server`。
- **采集/回放中 CAN 或底盘未就绪**：确认已执行对应 CAN 脚本（脚本内会自动执行），并确认 `galaxea_robot_startup.sh` 路径与 Galaxea 安装一致。
- **VR 窗格报错**：检查 Quest 与 ADB 连接，必要时在 quest 目录执行 `bash bash/adb.sh`。
- **VR 可以操控机械臂但无法操控躯干和底盘**： 1. 查看 `HDAS` 模块是否掉线,电量过低会导致 `HDAS` 模块掉线, 此时需充电并重启  2. 通过 `ip link list` 查看 `can0` 是否掉线, 运行 `bash frist_start/install.sh` 重装驱动
