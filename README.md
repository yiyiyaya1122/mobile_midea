# mobile_midea 🤖

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros) ![Python](https://img.shields.io/badge/Python-3.10.12-green?logo=python) ![License](https://img.shields.io/badge/License-TODO-orange)

mobile_midea 是一个基于 ROS 2 的高性能机器人数据收集与控制系统，专为机器人操作数据的同步采集、存储与控制而设计。支持多传感器数据收集、实时推理、机械臂控制和 VR 交互，数据以 HDF5 格式高效存储。

## 主要特性

- **多传感器数据收集**：支持 RealSense 摄像头、UVC 摄像头、激光雷达等多种传感器
- **实时推理**：支持 ARX 和 YAM 机械臂的 AI 推理控制
- **VR 交互**：集成 Quest 3/3S VR 设备，支持手势控制和实时反馈
- **机械臂控制**：支持 ARX R5 和 YAM 双机械臂控制
- **一键启动**：提供便捷的一键启动脚本，自动配置所有组件
- **CAN 总线支持**：完整的 CAN 总线配置和管理

# 项目结构

```
mobile_midea/
├── robot_msgs/                   # 共享消息定义包（ROS 2 包）
│   ├── msg/                      # 消息定义文件
│   │   ├── arx/                  # ARX 机械臂消息
│   │   ├── yam/                  # YAM 机械臂消息
│   │   ├── quest/                # Quest VR 消息
│   │   └── galaxea/              # Galaxea 底盘消息
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
├── prototype/                    # 主控制模块（ROS 2 工作空间）
│   ├── bash/                     # 系统脚本和启动脚本
│   ├── CAN/                      # CAN 总线配置（统一目录结构）
│   │   ├── ARX/                  # ARX CAN 配置
│   │   ├── YAM/                  # YAM CAN 配置
│   │   └── Galaxea/              # Galaxea CAN 配置
│   ├── config/                   # 摄像头配置文件
│   ├── scripts/                  # Python 工具脚本
│   ├── src/                      # ROS 2 工作空间源代码
│   │   ├── arx/                  # ARX 设备相关包
│   │   ├── yam/                  # YAM 设备相关包
│   │   ├── data_collector/       # 数据收集模块
│   │   ├── inference/            # 推理模块
│   │   ├── robot_msgs -> ../../robot_msgs  # 符号链接到共享消息包
│   │   └── rs_cameras/           # RealSense 摄像头驱动
│   ├── build/                    # 构建输出目录
│   ├── install/                  # 安装目录
│   └── README.md                 # 详细文档
├── quest/                        # Quest VR 模块（ROS 2 工作空间）
│   ├── bash/                     # ADB 和工具脚本
│   │   ├── adb.sh                # ADB 设备配置（自动检测）
│   │   ├── make_cert.sh          # HTTPS 证书生成
│   │   └── onestep_setadb.sh     # 一键配置 ADB
│   ├── src/                      # ROS 2 工作空间源代码
│   │   ├── quest3s_vr/           # Quest3S VR 主包
│   │   ├── yam_description/       # YAM 机械臂描述文件
│   │   ├── arx_description/      # ARX 机械臂描述文件
│   │   └── robot_msgs -> ../../robot_msgs  # 符号链接到共享消息包
│   ├── build/                    # 构建输出目录
│   ├── install/                  # 安装目录
│   └── README.md                 # 详细文档
├── quick_start/                  # 一键启动脚本
│   ├── quick_collect.sh          # 数据收集启动脚本（tmux 3x2 布局）
│   ├── quick_replay.sh           # 数据回放启动脚本（tmux 2x2 布局）
│   └── galaxea_robot_startup.sh  # galaxea_robot_startup 包装脚本
└── README.md                     # 本文件
```

## 主要模块说明

### robot_msgs/ - 共享消息定义包
统一的 ROS 2 消息定义包，包含所有机器人接口的消息类型：
- **ARX 消息**：`RobotCmd`、`RobotStatus`、`PosCmd`
- **YAM 消息**：`YamCmd`、`YamStatus`
- **Quest VR 消息**：`Quest2Button`
- **Galaxea 消息**：`MotorControl`

该包通过符号链接同时存在于 `prototype` 和 `quest` 工作空间中，确保消息定义的一致性。

详细文档请参考：[robot_msgs/README.md](robot_msgs/README.md)

### prototype/ - 主控制模块
包含完整的机器人控制系统：
- **数据收集**：多传感器数据同步采集和 HDF5 存储
  - 支持 chassis 和 torso target 速度数据收集
  - 支持图像、关节状态、底盘/躯干速度等多种数据类型
- **推理模块**：ARX 和 YAM 机械臂的 AI 推理控制
  - `joint_base_node`：关节和底盘状态/命令传输，支持 chassis 和 torso 数据推理
  - `camera_node`：相机图像传输到远程推理服务器
  - `data_replay_node`：HDF5 数据回放节点，用于在没有推理机时验证数据有效性
    - 使用 `replay_config.yaml` 配置文件
    - 发布命令 topic（`/YAM_VR_L`、`/YAM_VR_R` 或 `/ARX_INFER_L`、`/ARX_INFER_R`）而不是状态 topic
    - 不发布相机数据，仅发布关节命令和底盘/躯干目标速度
    - 支持 `arm_type` 参数（`yam` 或 `arx`）
- **机械臂控制**：ARX R5 和 YAM 双机械臂控制器
- **CAN 总线**：统一的 CAN 配置管理（ARX、YAM、Galaxea）
- **摄像头驱动**：RealSense 和 UVC 摄像头支持

详细文档请参考：[prototype/README.md](prototype/README.md)

### quest/ - Quest VR 模块
Quest 3/3S VR 设备集成：
- **VR 数据采集**：通过 HTTPS WebSocket 实时采集位置、姿态和按钮数据
- **机械臂可视化**：YAM 和 ARX 机械臂的 URDF 描述和 RViz 配置
- **ADB 自动配置**：自动检测设备并配置端口映射

详细文档请参考：[quest/README.md](quest/README.md)

### quick_start/ - 一键启动
提供便捷的一键启动功能：
- **`quick_collect.sh`**：数据收集启动脚本
  - tmux 3x2 布局（6 个 pane：CAN、VR、ARM、DATA、Camera、Rviz2）
  - 自动配置 CAN 接口、ADB 设备等
  - 启动完整的数据收集环境
- **`quick_replay.sh`**：数据回放启动脚本
  - tmux 2x2 布局（4 个 pane：DATA_REPLAY、INFERENCE、CAMERA、RVIZ2）
  - 支持参数化配置（HDF5 文件、配置文件、循环播放等）
  - 用于验证采集数据的有效性
- **状态监控**：实时显示各组件启动状态

# 安装指南

## 系统要求

| 组件         | 版本要求           | 推荐版本                   |
| ---------- | -------------- | ---------------------- |
| **操作系统**   | Ubuntu Linux   | Ubuntu 22.04 LTS       |
| **ROS 2**  | Humble 或更高     | ROS 2 Humble Hawksbill |
| **Python** | ≥3.8           | Python 3.10.12         |
| **CPU架构**  | x86_64 / ARM64 | x86_64                 |

## 安装 Pixi

[Pixi](https://pixi.sh/) 是一个快速的 Python 包和环境管理工具。本项目使用 Pixi 来管理 ROS 2 和 Python 依赖。

### 方法一：使用官方安装脚本（推荐）

```bash
curl -fsSL https://pixi.sh/install.sh | bash
source ~/.bashrc  # 或 source ~/.zshrc（如果使用 zsh）
```

### 方法二：手动安装

如果官方安装脚本不可用，可以手动安装：

```bash
# 下载 pixi（适用于 x86_64 Linux）
curl -L --insecure \
  -o pixi.tar.gz \
  https://github.com/prefix-dev/pixi/releases/latest/download/pixi-x86_64-unknown-linux-musl.tar.gz

# 验证下载的文件
file pixi.tar.gz

# 解压并安装
tar -xzf pixi.tar.gz
mkdir -p ~/.pixi/bin
mv pixi ~/.pixi/bin/pixi
chmod +x ~/.pixi/bin/pixi

# 添加到 PATH（如果尚未添加）
if ! grep -q '\.pixi/bin' ~/.bashrc; then
    echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
fi

# 重新加载配置
source ~/.bashrc

# 清理临时文件
rm -f pixi.tar.gz
```

### 验证 Pixi 安装

```bash
pixi --version
```

如果显示版本号，说明安装成功。

## 安装项目依赖

安装完 Pixi 后，需要分别安装各个子模块的依赖：

```bash
# 安装 prototype 模块依赖
cd prototype
pixi install

# 安装 quest 模块依赖
cd ../quest
pixi install
```

这将自动安装 ROS 2 Humble、Python 包以及其他所有项目依赖。

## 构建项目

安装完依赖后，需要分别构建各个子模块。`robot_msgs` 包已通过符号链接添加到 `prototype` 和 `quest` 工作空间中，`colcon` 会自动处理依赖关系，先编译 `robot_msgs`：

```bash
# 构建 prototype 模块（会自动先编译 robot_msgs）
cd prototype
pixi shell
colcon build --symlink-install
source install/setup.bash

# 构建 quest 模块（会自动先编译 robot_msgs）
cd ../quest
pixi shell
colcon build --symlink-install
source install/setup.bash
```

**注意**：如果 `robot_msgs` 符号链接不存在，可以手动创建：
```bash
# 在 prototype 工作空间
cd prototype/src
ln -sf ../../robot_msgs robot_msgs

# 在 quest 工作空间
cd quest/src
ln -sf ../../robot_msgs robot_msgs
```

# 使用说明

在使用项目功能前，请确保已激活对应模块的 pixi 环境：

```bash
# 对于 prototype 模块
cd prototype
pixi shell
source install/setup.bash

# 对于 quest 模块
cd quest
pixi shell
source install/setup.bash
```

## 快速开始

### 🚀 一键启动（推荐）

使用一键启动脚本自动配置和启动所有组件：

```bash
bash quick_start/quick_collect.sh
```

该脚本会创建 3 行 2 列的 tmux 布局，自动启动：
1. **CAN**（左上）：自动配置 Galaxea CAN 接口并启动底层驱动
2. **VR**（右上）：自动检测并配置 ADB 设备，启动 VR ROS2 节点
3. **ARM**（左中）：自动配置 YAM CAN 接口，启动 ARM 控制节点
4. **DATA**（右中）：启动数据收集节点
5. **Camera**（左下）：启动 UVC 摄像头节点
6. **Rviz2**（右下）：启动 Rviz2 可视化工具

所有组件会在 tmux 的不同 pane 中运行，方便监控和管理。

### 1. CAN 总线配置

```bash
# ARX CAN 总线
bash prototype/CAN/ARX/onestep_setcan.bash

# YAM CAN 总线
bash prototype/CAN/YAM/activate_yam_can.sh

# Galaxea CAN 总线
bash prototype/CAN/Galaxea/activate_galaxea_can.sh
```

### 2. ADB 设备配置（Quest VR）

```bash
# 自动检测设备并配置端口映射
bash quest/bash/adb.sh
```

### 3. 数据收集

```bash
cd prototype
pixi shell
source install/setup.bash

# 使用 launch 文件
ros2 launch data_collector data_collector.launch.py

# 或使用便捷脚本
bash bash/run_data_collector.sh
```

### 4. 推理

启动推理节点（ARX 或 YAM）：

```bash
cd prototype
pixi shell
source install/setup.bash

# ARX 推理
ros2 launch inference infer_arx.launch.py
# 或
bash bash/infer_arx.sh

# YAM 推理
ros2 launch inference infer_yam.launch.py
# 或
bash bash/infer_yam.sh
```

**推理模块功能**：
- 支持机械臂关节状态和命令传输
- 支持底盘和躯干速度数据推理
- 支持相机图像传输到远程推理服务器
- 通过 ZMQ 与远程推理服务器通信

### 4.1. 数据回放（验证数据有效性）

当没有推理机进行推理时，可以使用数据回放功能验证采集的数据：

#### 🚀 一键启动（推荐）

使用一键启动脚本快速启动数据回放环境：

```bash
# 基本用法
bash quick_start/quick_replay.sh /path/to/collection_xxx.hdf5

# 完整参数
bash quick_start/quick_replay.sh \
    /path/to/collection_xxx.hdf5 \
    /path/to/replay_config.yaml \
    true \
    yam
```

该脚本会创建 2 行 2 列的 tmux 布局，自动启动：
1. **DATA_REPLAY**（左上）：数据回放节点
2. **INFERENCE**（右上）：推理节点（接收回放数据）
3. **CAMERA**（左下）：UVC 摄像头节点
4. **RVIZ2**（右下）：可视化工具

#### 手动启动

```bash
cd prototype
pixi shell
source install/setup.bash

# 使用 launch 文件（默认使用 replay_config.yaml）
ros2 launch inference data_replay.launch.py \
    hdf5_file:=/path/to/collection_xxx.hdf5 \
    loop:=false \
    arm_type:=yam

# 指定配置文件路径
ros2 launch inference data_replay.launch.py \
    hdf5_file:=/path/to/collection_xxx.hdf5 \
    config_path:=/path/to/replay_config.yaml \
    loop:=false \
    arm_type:=yam

# 或直接运行节点
ros2 run inference data_replay_node \
    --ros-args \
    -p hdf5_file:=/path/to/collection_xxx.hdf5 \
    -p config_path:=/path/to/replay_config.yaml \
    -p loop:=false \
    -p arm_type:=yam

# 使用服务控制回放
ros2 service call /start_replay std_srvs/srv/Trigger
ros2 service call /stop_replay std_srvs/srv/Trigger
```

**功能特性**：
- 按原始采集频率回放数据
- 支持循环播放（设置 `loop:=true`）
- 自动将关节位置数据转换为命令消息（`YamCmd` 或 `RobotCmd`）
- 发布到命令 topic（`/YAM_VR_L`、`/YAM_VR_R` 或 `/ARX_INFER_L`、`/ARX_INFER_R`）
- 不发布相机数据，仅发布关节命令和底盘/躯干目标速度
- 支持 `arm_type` 参数选择机械臂类型（`yam` 或 `arx`）
- 使用 `replay_config.yaml` 配置文件（默认路径：`inference/config/replay_config.yaml`）
- 可与推理节点配合，验证完整的数据流

### 5. 摄像头控制

```bash
cd prototype
pixi shell

# UVC 摄像头
python scripts/uvc_cameras.py

# RealSense 摄像头（D405/D457/L515）
python scripts/d405_high.py      # D405 高位摄像头
python scripts/d405_wrist.py     # D405 腕部摄像头
python scripts/d457_high.py      # D457 高位摄像头
python scripts/d435_wrist.py     # D435 腕部摄像头
python scripts/launch_l515.py     # L515 激光雷达
```

### 6. 设备控制

```bash
cd prototype
pixi shell
source install/setup.bash

# ARX 设备
bash bash/run_arx.sh

# YAM 设备
bash bash/run_yam.sh

# 接管 YAM 控制
bash bash/take_over_yam.sh
```

### 7. VR 控制器

```bash
cd quest
pixi shell
source install/setup.bash

# 启动 VR 控制器节点
ros2 launch quest3s_vr vr_control.launch.py
```

# 开发指南

## 环境设置

开发前，请确保已激活对应模块的 pixi 环境：

```bash
# prototype 模块
cd prototype
pixi shell
source install/setup.bash

# quest 模块
cd quest
pixi shell
source install/setup.bash
```

## 构建特定包

在开发过程中，可以只构建特定的包以节省时间。如果包依赖于 `robot_msgs`，建议先编译 `robot_msgs`：

```bash
# prototype 模块
cd prototype
pixi shell
# 先编译 robot_msgs（如果依赖它）
colcon build --packages-select robot_msgs
# 然后编译目标包
colcon build --packages-select data_collector

# quest 模块
cd quest
pixi shell
# 先编译 robot_msgs（如果依赖它）
colcon build --packages-select robot_msgs
# 然后编译目标包
colcon build --packages-select quest3s_vr
```

或者直接编译所有包，`colcon` 会自动处理依赖顺序：
```bash
colcon build --symlink-install
```

## 常用开发命令

```bash
# 查看 ROS 2 节点
ros2 node list

# 查看话题
ros2 topic list

# 查看服务
ros2 service list

# 监听话题数据
ros2 topic echo /topic_name

# 查看节点信息
ros2 node info /node_name
```

## 项目配置

- **ROS 2 实现**：FastRTPS（在 `pixi.toml` 中配置）
- **Python 版本**：3.8 - 3.10
- **主要依赖**：ROS 2 Humble、RealSense SDK、NumPy、SciPy 等

# 故障排除

## 常见问题

### Pixi 命令未找到

如果提示 `pixi: command not found`，请确保：
1. 已正确安装 pixi
2. PATH 环境变量已配置：`export PATH="$HOME/.pixi/bin:$PATH"`
3. 已重新加载 shell 配置：`source ~/.bashrc`

### ROS 2 节点无法启动

1. 确保已激活对应模块的 pixi 环境：`pixi shell`
2. 检查 ROS 2 环境是否已 source：
   ```bash
   source install/setup.bash
   ```
3. 确认所有依赖已安装：`pixi install`
4. 确认 `robot_msgs` 已编译：如果节点依赖 `robot_msgs`，确保它已成功编译

### robot_msgs 编译失败

如果 `robot_msgs` 编译失败：

1. 检查符号链接是否存在：
   ```bash
   ls -la prototype/src/robot_msgs
   ls -la quest/src/robot_msgs
   ```

2. 如果符号链接不存在，手动创建：
   ```bash
   cd prototype/src
   ln -sf ../../robot_msgs robot_msgs
   cd ../../quest/src
   ln -sf ../../robot_msgs robot_msgs
   ```

3. 检查 `robot_msgs/package.xml` 中的 email 地址格式是否正确（必须是有效的 email 格式）

### 摄像头设备权限问题

如果遇到摄像头权限错误，运行：

```bash
cd prototype
# RealSense 摄像头
bash bash/realsense-rules.sh

# UVC 摄像头
sudo cp bash/99-uvc-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

### CAN 总线设备问题

确保已正确配置 CAN 总线：

```bash
# ARX CAN
bash prototype/CAN/ARX/onestep_setcan.bash

# YAM CAN
bash prototype/CAN/YAM/activate_yam_can.sh

# Galaxea CAN
bash prototype/CAN/Galaxea/activate_galaxea_can.sh
```

### ADB 设备连接问题

如果 Quest 设备无法连接：

```bash
# 检查设备连接
adb devices

# 自动配置 ADB（自动检测设备）
bash quest/bash/adb.sh

# 手动配置（如果需要）
bash quest/bash/onestep_setadb.sh
```

### 一键启动脚本问题

如果一键启动失败：

1. 检查依赖是否安装：`tmux`、`pixi`、`adb`
2. 检查路径是否正确：确保 `prototype` 和 `quest` 目录存在
3. 检查权限：确保脚本有执行权限
4. 查看 tmux pane 中的错误信息

# 贡献

如果您想为该项目做出贡献，请遵循以下步骤：

1. Fork 本仓库
2. 创建一个新的分支：`git checkout -b feature-branch`
3. 提交更改：`git commit -m '描述更改内容'`
4. 推送分支：`git push origin feature-branch`
5. 创建 Pull Request

## 代码规范

- 遵循 PEP 8 Python 代码规范
- 使用有意义的变量和函数名
- 添加必要的注释和文档字符串
- 确保代码通过 lint 检查

# 许可证

TODO
