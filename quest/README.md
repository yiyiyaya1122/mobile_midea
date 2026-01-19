# Quest3S VR 🤖

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros) ![Python](https://img.shields.io/badge/Python-3.10-green?logo=python) ![License](https://img.shields.io/badge/License-TODO-orange)

Quest 是一个基于 ROS 2 的 Quest 3/3S VR 与机械臂控制集成示例工程。支持 VR 数据采集、HTTPS WebSocket 服务、机械臂描述与可视化，以及自定义 ROS 消息定义。

## 主要特性

- **VR 数据采集**：通过 HTTPS WebSocket 服务实时采集 Quest 3/3S VR 设备的位置、姿态和按钮数据
- **机械臂可视化**：提供 YAM 和 ARX 机械臂的 URDF 描述文件和 RViz 启动配置
- **自定义消息**：定义完整的 ROS 2 消息类型，支持机械臂控制和状态反馈
- **便捷工具**：提供 ADB 连接、证书生成等实用脚本

# 项目结构

```
quest/
├── bash/                          # 系统脚本和工具脚本
│   ├── adb.sh                     # ADB 端口反向映射脚本
│   ├── make_cert.sh               # HTTPS 证书生成脚本
│   ├── onestep_setadb.sh          # 一键配置 ADB 脚本
│   └── replace.sh                 # 文本替换工具脚本
├── scripts/                       # Python 工具脚本
│   └── package_xml_2_cmake.py    # package.xml 转 CMakeLists.txt 工具
├── src/                           # ROS 2 工作空间源代码
│   ├── quest3s_vr/               # Quest3S VR 主包（Python）
│   │   ├── config/                # 配置文件目录（证书等）
│   │   ├── quest3s_vr/           # Python 包
│   │   │   ├── controller_node.py # VR 控制器主节点
│   │   │   ├── controllers.py    # 控制器实现
│   │   │   ├── hand_pub.py       # 手部数据发布节点
│   │   │   └── hands.py          # 手部数据处理
│   │   ├── resource/             # 资源文件
│   │   ├── setup.py              # Python 包安装配置
│   │   └── package.xml           # ROS 2 包定义
│   ├── yam_description/           # YAM 机械臂描述文件
│   │   ├── launch/               # Launch 文件
│   │   │   └── display.launch.py # RViz 显示启动文件
│   │   ├── meshes/               # 3D 模型文件（STL）
│   │   ├── rviz/                 # RViz 配置文件
│   │   │   └── yam.rviz         # YAM 可视化配置
│   │   ├── urdf/                 # URDF 描述文件
│   │   │   └── yam.urdf         # YAM 机械臂 URDF
│   │   └── package.xml           # ROS 2 包定义
│   ├── arx_description/          # ARX 机械臂描述文件
│   │   ├── meshes/               # 3D 模型文件（STL）
│   │   │   └── base_link.STL ~ link8.STL
│   │   └── r5.urdf               # ARX R5 机械臂 URDF
│   └── msg/                      # 自定义 ROS 2 消息定义
│       ├── arxmsgros2/          # ARX 消息包
│       │   ├── arm_control/     # 手臂控制消息
│       │   │   └── msg/PosCmd.msg
│       │   └── arx5_arm_msg/    # ARX5 机械臂消息
│       │       └── msg/
│       │           ├── RobotCmd.msg
│       │           └── RobotStatus.msg
│       ├── quest2_button_msg/   # Quest2 按钮消息
│       │   └── msg/Quest2Button.msg
│       └── yam_arm_msg/         # YAM 机械臂消息
│           └── msg/
│               ├── YamCmd.msg
│               └── YamStatus.msg
├── build/                        # 构建输出目录（由 colcon 生成）
├── install/                      # 安装目录（由 colcon 生成）
├── log/                          # 构建日志目录（由 colcon 生成）
├── pixi.toml                     # Pixi 项目配置文件
│                                # 声明依赖、channels、tasks、环境变量等
├── pixi.lock                     # Pixi 锁定文件（自动生成）
│                                # 记录精确的依赖版本与哈希，保证环境一致性
└── README.md                     # 项目说明文档
```

## 主要模块说明

### 系统脚本 (`bash/`)
包含系统级脚本，用于设备连接和配置：
- **ADB 脚本**：`adb.sh`、`onestep_setadb.sh` - 用于连接 Quest 设备并配置端口映射
- **证书生成**：`make_cert.sh` - 生成 HTTPS WebSocket 所需的 SSL 证书
- **工具脚本**：`replace.sh` - 文本替换工具

### Python 工具脚本 (`scripts/`)
独立的 Python 工具脚本：
- **包转换工具**：`package_xml_2_cmake.py` - 将 package.xml 转换为 CMakeLists.txt

### ROS 2 工作空间 (`src/`)
包含所有 ROS 2 包的源代码：
- **`quest3s_vr/`**：Quest3S VR 主包，包含控制器节点、手部数据发布等
  - `controller_node.py`：VR 控制器主节点，提供 HTTPS WebSocket 服务
  - `hand_pub.py`：手部数据发布节点
  - `controllers.py`、`hands.py`：控制器和手部数据处理实现
- **`yam_description/`**：YAM 机械臂 URDF 描述文件和 RViz 配置
- **`arx_description/`**：ARX 机械臂 URDF 描述文件
- **`msg/`**：自定义 ROS 2 消息类型定义
  - ARX 和 YAM 机械臂消息
  - Quest2 按钮消息

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
| **Python** | ≥3.9, <3.11    | Python 3.10            |
| **CPU架构**  | x86_64         | x86_64                 |

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

### 1. 生成 HTTPS 证书

HTTPS WebSocket 服务需要 SSL 证书。生成证书并复制到配置目录：

```bash
# 生成证书
./bash/make_cert.sh

# 复制到配置目录
mv cert.pem src/quest3s_vr/config/
mv key.pem  src/quest3s_vr/config/
```

### 2. 配置 ADB 连接（可选）

如果使用 Quest 设备，需要配置 ADB 连接：

1. 在 Quest 上开启开发者模式并授权 USB 调试
2. 使用脚本自动配置：

```bash
# 自动检测序列号并更新 adb.sh
bash bash/onestep_setadb.sh

# 将设备的端口反向映射到本机（默认 tcp:8012）
bash bash/adb.sh
```

### 3. 启动 VR 控制器节点

```bash
# 启动主控制器节点（提供 HTTPS WebSocket 服务）
ros2 run quest3s_vr controller_node

ros2 launch quest3s_vr vr_control.launch.py
```

### 4. 启动其他组件（可选）

```bash
# 启动手部数据发布节点
ros2 run quest3s_vr hand_pub

# 启动 YAM 机械臂可视化（RViz）
ros2 launch yam_description display.launch.py
```

## 节点与话题

### WebSocket 服务
- **地址**：`0.0.0.0:8012`
- **协议**：HTTPS (WSS)
- **静态资源路径**：`/static`

### 发布的话题

| 话题名称          | 消息类型              | 说明                    |
| -------------- | ----------------- | --------------------- |
| `/vr_pose_left` | `geometry_msgs/Pose` | 左手控制器位置和姿态            |
| `/vr_pose_right` | `geometry_msgs/Pose` | 右手控制器位置和姿态            |
| `/vr_pose_head` | `geometry_msgs/Pose` | 头部位置和姿态               |
| `/vr_button`    | `quest2_button_msg/Quest2Button` | Quest 按钮状态           |

### 订阅的话题

| 话题名称          | 消息类型              | 说明                    |
| -------------- | ----------------- | --------------------- |
| `/arm_l_status` | `yam_arm_msg/YamStatus` 或 `arx5_arm_msg/RobotStatus` | 左机械臂状态反馈 |
| `/arm_r_status` | `yam_arm_msg/YamStatus` 或 `arx5_arm_msg/RobotStatus` | 右机械臂状态反馈 |

## 快速调试命令

```bash
# 列出所有话题
ros2 topic list

# 查看话题数据
ros2 topic echo /vr_pose_left
ros2 topic echo /vr_pose_right
ros2 topic echo /vr_button

# 查看节点信息
ros2 node list
ros2 node info /controller_node

# 查看话题信息
ros2 topic info /vr_pose_left
ros2 topic hz /vr_pose_left

# 释放被占用端口（示例 8012）
fuser -k 8012/tcp || lsof -i:8012 -t | xargs -r kill -9
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
colcon build --packages-select quest3s_vr

# 构建多个指定包
colcon build --packages-select quest3s_vr yam_description
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

# 查看话题类型
ros2 topic type /topic_name

# 查看消息定义
ros2 interface show geometry_msgs/Pose
```

## 项目配置

- **ROS 2 实现**：FastRTPS（在 `pixi.toml` 中配置）
- **Python 版本**：3.9 - 3.11（推荐 3.10）
- **主要依赖**：ROS 2 Humble、vuer、NumPy、transforms3d、scipy、pinocchio、msgpack 等

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

### 端口被占用

如果端口 8012 被占用，可以使用以下命令释放：

```bash
# 方法一：使用 fuser
fuser -k 8012/tcp

# 方法二：使用 lsof
lsof -i:8012 -t | xargs -r kill -9

# 方法三：强制关闭残留进程
pkill -f controller_node
```

### ADB 权限问题

如果遇到 ADB 权限错误：

```bash
# 方法一：创建或更新 udev 规则
# 创建 /etc/udev/rules.d/51-android.rules 文件
# 添加设备规则（需要根据实际设备 ID 修改）

# 方法二：临时使用 sudo
sudo adb start-server

# 方法三：将用户添加到 plugdev 组
sudo usermod -aG plugdev $USER
# 然后重新登录
```

### 证书问题

如果 HTTPS WebSocket 连接失败：

1. 检查证书文件是否存在：
   ```bash
   ls -l src/quest3s_vr/config/cert.pem
   ls -l src/quest3s_vr/config/key.pem
   ```

2. 重新生成证书：
   ```bash
   ./bash/make_cert.sh
   mv cert.pem src/quest3s_vr/config/
   mv key.pem src/quest3s_vr/config/
   ```

### 依赖包版本冲突

如果遇到 Python 依赖包版本冲突：

1. 建议使用 Pixi 环境管理（推荐）：
   ```bash
   pixi shell
   ```

2. 如果使用独立 Python 虚拟环境：
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade 'vuer[all]' numpy transforms3d scipy pinocchio msgpack
   ```

3. 若 NumPy/SciPy 显示版本警告，按需要锁定版本：
   ```bash
   pip install "numpy<1.25" scipy
   ```

### WebSocket 连接失败

如果 Quest 设备无法连接到 WebSocket 服务：

1. 检查防火墙设置，确保端口 8012 开放
2. 确认 ADB 端口映射已正确配置：
   ```bash
   adb devices
   adb reverse tcp:8012 tcp:8012
   ```
3. 检查 Quest 设备上的网络设置，确保可以访问开发机 IP

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
- 提交 PR 时确保代码风格一致并包含必要说明

# 许可证

本仓库未包含特定许可证（若需发布到公共仓库，请补充 LICENSE 文件）。如有问题或建议，请在仓库 Issue 中联系维护者。
