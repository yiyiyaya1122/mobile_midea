# Prototype 🤖

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros) ![Python](https://img.shields.io/badge/Python-3.10.12-green?logo=python) ![License](https://img.shields.io/badge/License-TODO-orange)

Prototype 是一个基于 ROS 2 的高性能机器人数据收集与控制系统，专为机器人操作数据的同步采集、存储与控制而设计。支持多传感器数据收集、实时推理和机械臂控制，数据以 HDF5 格式高效存储。

# 项目结构

```
prototype/
├── bash/                          # 系统脚本和启动脚本
│   ├── run_arx.sh                 # ARX 设备启动脚本
│   ├── run_yam.sh                 # YAM 设备启动脚本
│   ├── run_data_collector.sh      # 数据收集启动脚本
│   ├── run_cameras.sh             # 摄像头启动脚本
│   ├── run_uvc_cameras.sh          # UVC 摄像头启动脚本
│   ├── run_aloha_cameras.sh       # ALOHA 摄像头启动脚本
│   ├── infer_arx.sh               # ARX 推理启动脚本
│   ├── infer_yam.sh               # YAM 推理启动脚本
│   ├── take_over_yam.sh           # 接管 YAM 控制脚本
│   ├── activate_can.sh            # CAN 总线激活脚本
│   ├── install_communication_tools.sh  # 通信工具安装脚本
│   ├── realsense-rules.sh         # RealSense 权限规则安装
│   ├── 99-realsense-libusb.rules  # RealSense USB 权限规则
│   └── 99-uvc-cameras.rules       # UVC 摄像头权限规则
├── CAN_ARX/                        # ARX CAN 总线配置
│   ├── arx_can/                   # CAN 通道配置脚本
│   │   ├── arx_can0.sh ~ arx_can5.sh  # 各通道配置脚本
│   ├── onestep_setcan.bash        # 一键配置脚本
│   ├── arx_can.rules              # CAN 总线权限规则
│   ├── search.sh                  # CAN 设备搜索脚本
│   ├── set.sh                     # CAN 参数设置脚本
│   └── test_protect.sh            # 保护测试脚本
├── CAN_YAM/                        # YAM CAN 总线配置
│   ├── activate_yam_can.sh        # YAM CAN 激活脚本
│   ├── clear_yam_rules.sh         # 清除 YAM 规则脚本
│   └── map_yam_can.sh             # YAM CAN 映射脚本
├── config/                         # 摄像头配置文件
│   ├── d405.json                  # D405 摄像头配置
│   ├── d457.yaml                  # D457 摄像头配置
│   └── l515.json                  # L515 激光雷达配置
├── scripts/                        # Python 工具脚本
│   ├── uvc_cameras.py             # UVC 摄像头控制
│   ├── d405_high.py               # D405 高位摄像头脚本
│   ├── d405_wrist.py              # D405 腕部摄像头脚本
│   ├── d435_wrist.py              # D435 腕部摄像头脚本
│   ├── d457_high.py               # D457 高位摄像头脚本
│   ├── launch_l515.py             # L515 激光雷达启动脚本
│   └── data_statistics.py         # 数据统计工具
├── src/                            # ROS 2 工作空间源代码
│   ├── arx/                       # ARX 设备相关包
│   │   ├── arx_cmd_pub/           # ARX 命令发布节点（Python）
│   │   └── arx_r5_controller/     # ARX R5 控制器（C++）
│   │       ├── config/            # 控制器配置文件
│   │       ├── launch/            # Launch 文件
│   │       ├── include/          # 头文件
│   │       └── src/              # 源文件
│   ├── data_collector/            # 数据收集模块（Python）
│   │   ├── config/                # 数据收集配置
│   │   ├── launch/                # Launch 文件
│   │   └── data_collector/        # Python 包
│   ├── inference/                 # 推理模块（Python）
│   │   ├── launch/                # Launch 文件
│   │   └── inference/            # Python 包
│   ├── msg/                       # 自定义 ROS 2 消息定义
│   │   ├── arxmsgros2/           # ARX 消息包
│   │   │   ├── arm_control/      # 手臂控制消息
│   │   │   └── arx5_arm_msg/     # ARX5 机械臂消息
│   │   ├── quest2_button_msg/    # Quest2 按钮消息
│   │   └── yam_arm_msg/          # YAM 机械臂消息
│   ├── rs_cameras/                # RealSense 摄像头驱动
│   │   └── realsense-ros-4.51.1/  # RealSense ROS 2 驱动包
│   │       ├── realsense2_camera/      # 摄像头节点
│   │       ├── realsense2_camera_msgs/ # 消息定义
│   │       └── realsense2_description/ # URDF 描述文件
│   └── yam/                       # YAM 设备相关包
│       ├── yam_cmd_pub/           # YAM 命令发布节点（Python）
│       ├── yam_damiao_controller/ # YAM 控制器（C++）
│       └── yam_description/       # YAM URDF 描述文件
├── build/                          # 构建输出目录（由 colcon 生成）
├── install/                        # 安装目录（由 colcon 生成）
├── log/                            # 构建日志目录（由 colcon 生成）
├── pixi.toml                       # Pixi 项目配置文件
│                                  # 声明依赖、channels、tasks、环境变量等
├── pixi.lock                       # Pixi 锁定文件（自动生成）
│                                  # 记录精确的依赖版本与哈希，保证环境一致性
└── README.md                       # 项目说明文档
```

## 主要模块说明

### 系统脚本 (`bash/`)
包含系统级脚本，用于启动各种服务和配置设备权限：
- **设备启动脚本**：`run_arx.sh`、`run_yam.sh`、`run_data_collector.sh` 等
- **推理脚本**：`infer_arx.sh`、`infer_yam.sh`
- **摄像头脚本**：`run_cameras.sh`、`run_uvc_cameras.sh` 等
- **权限规则**：RealSense 和 UVC 摄像头的 udev 规则文件

### CAN 总线配置 (`CAN_ARX/`, `CAN_YAM/`)
CAN 总线配置和映射脚本：
- **ARX CAN**：包含多通道配置脚本和权限规则
- **YAM CAN**：包含激活、清除和映射脚本

### 配置文件 (`config/`)
各种摄像头设备的配置文件，包括 D405、D457 和 L515 的 JSON/YAML 配置。

### Python 工具脚本 (`scripts/`)
独立的 Python 工具脚本：
- **摄像头控制**：UVC、D405、D435、D457 摄像头脚本
- **激光雷达**：L515 启动脚本
- **数据分析**：`data_statistics.py` 数据统计工具

### ROS 2 工作空间 (`src/`)
包含所有 ROS 2 包的源代码：
- **`arx/`**：ARX 机械臂控制相关节点
  - `arx_cmd_pub/`：Python 命令发布节点
  - `arx_r5_controller/`：C++ 控制器，包含配置、launch 文件和源代码
- **`data_collector/`**：数据收集和记录功能，支持 HDF5 格式存储
- **`inference/`**：AI 推理节点，支持 ARX 和 YAM 设备
- **`msg/`**：自定义 ROS 2 消息类型定义
  - ARX 和 YAM 机械臂消息
  - Quest2 按钮消息
- **`rs_cameras/`**：Intel RealSense 摄像头 ROS 2 驱动包
- **`yam/`**：YAM 设备控制相关节点
  - `yam_cmd_pub/`：Python 命令发布节点
  - `yam_damiao_controller/`：C++ 控制器
  - `yam_description/`：URDF 描述文件

### 构建产物目录
- **`build/`**：colcon 构建输出目录（由构建系统自动生成）
- **`install/`**：安装目录，包含编译后的可执行文件和库（由 colcon 自动生成）
- **`log/`**：构建日志目录（由 colcon 自动生成）

### 项目配置文件
- **`pixi.toml`**：Pixi 项目配置文件，声明依赖、channels、tasks 和环境变量
- **`pixi.lock`**：Pixi 锁定文件（自动生成），记录精确的依赖版本与哈希，确保不同环境的一致性

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

安装完 Pixi 后，在项目根目录运行以下命令安装所有依赖：

```bash
pixi install
```

这将自动安装 ROS 2 Humble、Python 包以及其他所有项目依赖。

## 构建项目

安装完依赖后，需要构建项目。首先激活 pixi 环境，然后使用 colcon 构建：

```bash
# 激活 pixi 环境
pixi shell

# 构建所有包
colcon build

# 或使用符号链接模式（推荐用于开发，修改代码后无需重新构建）
colcon build --symlink-install
```

构建完成后，source 工作空间：

```bash
source install/setup.bash
```

## 运行测试

构建完成后，可以运行测试以验证安装：

```bash
# 在 pixi shell 中运行所有测试
colcon test

# 查看测试结果
colcon test-result --verbose
```

# 使用说明

在使用项目功能前，请确保已激活 pixi 环境：

```bash
pixi shell
```

## 快速开始

### 1. 数据收集

启动数据收集节点：

```bash
# 使用 launch 文件
ros2 launch data_collector data_collector.launch.py

# 或使用便捷脚本
bash/run_data_collector.sh
```

### 2. 推理

启动推理节点（ARX 或 YAM）：

```bash
# ARX 推理
ros2 launch inference infer_arx.launch.py
# 或
bash/infer_arx.sh

# YAM 推理
ros2 launch inference infer_yam.launch.py
# 或
bash/infer_yam.sh
```

### 3. 摄像头控制

```bash
# UVC 摄像头
python scripts/uvc_cameras.py

# RealSense 摄像头（D405/D457/L515）
python scripts/d405_high.py      # D405 高位摄像头
python scripts/d405_wrist.py     # D405 腕部摄像头
python scripts/d457_high.py      # D457 高位摄像头
python scripts/d435_wrist.py     # D435 腕部摄像头
python scripts/launch_l515.py     # L515 激光雷达
```

### 4. 设备控制

```bash
# ARX 设备
bash bash/run_arx.sh

# YAM 设备
bash bash/run_yam.sh

# 接管 YAM 控制
bash bash/take_over_yam.sh
```

### 5. CAN 总线配置

```bash
# ARX CAN 总线
bash CAN_ARX/onestep_setcan.bash

# YAM CAN 总线
bash CAN_YAM/activate_yam_can.sh
```

# 开发指南

## 环境设置

开发前，请确保已激活 pixi 环境：

```bash
pixi shell
```

## 构建特定包

在开发过程中，可以只构建特定的包以节省时间：

```bash
# 构建特定包
colcon build --packages-select data_collector

# 构建多个指定包
colcon build --packages-select data_collector inference
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

1. 确保已激活 pixi 环境：`pixi shell`
2. 检查 ROS 2 环境是否已 source：
   ```bash
   source install/setup.bash
   ```
3. 确认所有依赖已安装：`pixi install`

### 摄像头设备权限问题

如果遇到摄像头权限错误，运行：

```bash
# RealSense 摄像头
bash/realsense-rules.sh

# UVC 摄像头
sudo cp bash/99-uvc-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

### CAN 总线设备问题

确保已正确配置 CAN 总线规则：

```bash
# ARX CAN
cd CAN_ARX && bash onestep_setcan.bash

# YAM CAN
cd CAN_YAM && bash activate_yam_can.sh
```

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

