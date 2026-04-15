# 使用环境

- wsl2 + ubuntu 24.04
- ros2 jazzy(desktop) + gazebo harmonic
- 使用ament-python参数通过colcon构建功能包等
- 使用python3.12.3开发，且使用目录下的venv虚拟环境

## 环境配置过程

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## 添加 Gazebo 官方源 (OSRF)

虽然 Ubuntu 仓库里有 Gazebo，但添加官方源能确保你获得 Harmonic 的最新补丁。

### 1. 下载并安装 Gazebo GPG 密钥

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
```

### 2. 将 Gazebo 仓库添加到源列表

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

## 安装gazebo

```bash
sudo apt install ros-jazzy-ros-gz -y
```

## 安装替代fastdds的中间件

用于解决wsl中fastdds使用共享内存无法通讯的问题

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp -y
```

## 创建venv虚拟环境并且使用requirements.txt安装依赖

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## 建议将以下写入~/.bashrc

```bash
export GALLIUM_DRIVER=d3d12 #enable GPU
export LIBGL_ALWAYS_SOFTWARE=0 #prefer GPU
source /opt/ros/jazzy/setup.bash
```
如果长期只有这一个工作空间，建议将env.sh也写入~/.bashrc

```bash
source ~/swarm-ros/env.sh
```

## 杂项
查看是否使用gpu：
root@Computer:~# glxinfo | grep "OpenGL renderer"
Dropped Escape call with ulEscapeCode : 0x03007703
OpenGL renderer string: D3D12 (AMD Radeon 780M Graphics)

验证图形化是否正常
glxgears

配置代码补全
https://github.com/akinomyoga/ble.sh