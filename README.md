# ur5-yolo-grasp

> UR5 + Robotiq 2F-85 + YOLO 视觉抓取仿真 — ROS2 Humble + Gazebo Harmonic + MoveIt2 全栈实现。

YOLO 检测桌面物体 → 深度查表 + 针孔投影获取 3D 位置 → MoveIt2 IK 解算 → JointTrajectoryController 执行 → Robotiq 闭合抓取 → 搬运到放置区。整个闭环在 Gazebo 仿真里跑，单类 YOLO 模型 + 位置去重逻辑保证 9 个物体依次被抓取。

## 环境要求

| 组件 | 版本 |
|---|---|
| Ubuntu | 22.04 |
| ROS2 | Humble |
| Gazebo | Harmonic |
| YOLO | ultralytics（系统 python3.10） |
| GPU | 推荐 NVIDIA（YOLO 默认 `cuda:0`；无 GPU 时运行命令加 `-p device:=cpu`） |

## 部署

### 0. 前置检查

> ⚠️ **如果你装了 Anaconda / Miniconda：整个部署过程不要激活任何 conda 环境。**
>
> ROS2 Humble 的 Python 组件（`catkin_pkg`、`ament_python`、`cv_bridge` 等）全部由 apt 装给系统 `/usr/bin/python3.10`，conda 的任何 python 版本都看不到它们。编译和运行时 conda 出现在 PATH 前面 = 必炸。
>
> **每条新终端都先确认**：`which python3` → `/usr/bin/python3`，不是 conda 路径。
> 如果不对：`export PATH=/usr/bin:$PATH`，或者 `conda deactivate`。

```bash
# 确认系统 python3.10 在 PATH 最前面
which python3
# /usr/bin/python3   ✅ 正确
# /home/<user>/miniconda3/bin/python3   ❌ 关掉 conda 或 export PATH=/usr/bin:$PATH

# 如果没有 pip（Ubuntu 22.04 minimal 默认不带）
/usr/bin/python3 -m pip --version || sudo apt install -y python3-pip
```

### 1. 安装系统依赖

```bash
# 添加 apt 仓库
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update
sudo apt install -y curl python3-pip git \
  ros-humble-desktop ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robotiq-description ros-humble-ur-description ros-humble-ur-moveit-config \
  ros-humble-ros-gzharmonic gz-harmonic
```

> 如果你的机器已经装过 ROS2 Humble + Gazebo Harmonic，可以跳过 apt 装包那行，但 **确保 `python3-pip` 和 `git` 已装**（`dpkg -l python3-pip git`）。

### 2. 安装 Python 依赖

```bash
# ⚠️ 必须用系统 python3 的 pip（不是 conda 的 pip）
# matplotlib>=3.8 跟 ultralytics 一起装：pip 会把 numpy 升到 2.x，apt 的 matplotlib 3.5 不兼容
/usr/bin/python3 -m pip install --user ultralytics opencv-python "matplotlib>=3.8"

# ultralytics 要求 setuptools≥70，pip 会自动装 83。
# 但 colcon 的 ament_python 不支持 setuptools 79+，必须锁回 68。
# 这一步不能跟上面合并——pip 不会同时接受 "≥70" 和 "==68"。
/usr/bin/python3 -m pip install --user "setuptools==68.0.0"
```

> 为什么是 68：79 已删除了 `canonicalize_version(strip_trailing_zero=…)` 这个 ament_python 还在用的参数，70–78 没测过。

### 3. 创建工作区

```bash
mkdir -p ~/grasp_ws/src && cd ~/grasp_ws/src

# 克隆 gz_ros2_control（必须源码编译，apt 版只支持 Gazebo Fortress）
git clone --depth 1 -b humble https://github.com/ros-controls/gz_ros2_control.git
# 如果报 HTTP/2 错误，改用 HTTP/1.1：
#   git -c http.version=HTTP/1.1 clone --depth 1 -b humble \
#       https://github.com/ros-controls/gz_ros2_control.git

# 克隆本仓库
git clone --depth 1 https://github.com/TUSZ-AGDR/ch4-yolo-ik-grasp.git
# （若仓库地址变了，用实际 URL 替换）

# 创建软链（colcon 要求 ROS2 包直接在 src/ 下）
ln -s ch4-yolo-ik-grasp/grasp_sim grasp_sim
ln -s ch4-yolo-ik-grasp/yolo_vision yolo_vision
```

### 4. 编译并首次启动

```bash
# —— 编译（⚠️ 确保 conda 不在 PATH 里，见第 0 步）——
cd ~/grasp_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
colcon build --symlink-install \
  --packages-skip ign_ros2_control ign_ros2_control_demos gz_ros2_control_demos gz_ros2_control_tests \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

# —— 预生成 URDF/SRDF（只需首次）——
xacro ~/grasp_ws/src/ch4-yolo-ik-grasp/grasp_sim/models/ur5_with_gripper.urdf.xacro \
  name:=ur5 ur_type:=ur5 safety_limits:=false sim_ignition:=true sim_gazebo:=false \
  simulation_controllers:=$(ros2 pkg prefix grasp_sim)/share/grasp_sim/config/ros2_control.yaml \
  -o /tmp/ur5.urdf
xacro ~/grasp_ws/src/ch4-yolo-ik-grasp/grasp_sim/config/ur5_gripper.srdf.xacro \
  name:=ur5 -o /tmp/ur5.srdf

# —— 一键启动 ——
export GZ_VERSION=harmonic GZ_SIM_RESOURCE_PATH="/opt/ros/humble/share"
ros2 launch grasp_sim full.launch.py 2>&1 | tee /tmp/launch.log
```

往后重启只需最后两句 `source` + `export` + `ros2 launch`。约 30 秒后 Gazebo 窗口弹出，机器人开始抓取。

### 附：拆开启动（3 终端，便于调试/演示）

`full.launch.py` 内部就是这 3 条命令。分开展示时各组件启动过程更清晰：

```bash
# 终端 1 — Gazebo + UR5 + MoveIt2（先启，等 ~15s 控制器就绪）
source /opt/ros/humble/setup.bash && source ~/grasp_ws/install/setup.bash
export GZ_VERSION=harmonic GZ_SIM_RESOURCE_PATH="/opt/ros/humble/share"
ros2 launch grasp_sim sim.launch.py 2>&1 | tee /tmp/launch.log

# 终端 2 — YOLO 检测（无 GPU 时命令末尾加 -p device:=cpu）
source /opt/ros/humble/setup.bash && source ~/grasp_ws/install/setup.bash
ros2 run yolo_vision detector --ros-args -p model:=best.pt -p confidence:=0.5

# 终端 3 — 抓取规划
source /opt/ros/humble/setup.bash && source ~/grasp_ws/install/setup.bash
ros2 run grasp_sim grasp_planner
```

## 验证启动

```bash
# mimic 同步生效（应看到 5 行）
grep "is mimicking" /tmp/launch.log

# P 增益已加载（输出 1 行或以上即可）
grep "proportional_gain" /tmp/launch.log

# IK 服务在线
ros2 service list | grep compute_ik
```

### 关闭

```bash
# 各终端 Ctrl+C，或强制清理所有相关进程：
pkill -f "ros2 launch grasp_sim"
pkill -f "ros2 run yolo_vision"
pkill -f "ros2 run grasp_sim"
pkill -f "gz sim"
pkill -f "move_group"
```

## 已知环境陷阱

| 陷阱 | 现象 | 修复 |
|---|---|---|
| **conda 抢 PATH** | `ModuleNotFoundError: No module named 'catkin_pkg'`（CMake 调了 conda 的 python，它看不到 apt 装的 ROS2 包） | `conda deactivate`，`export PATH=/usr/bin:$PATH`，`rm -rf build install log`，重编译 |
| **setuptools ≥ 79** | `canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'` | `/usr/bin/python3 -m pip install --user "setuptools==68.0.0"` |
| **matplotlib < 3.8 + numpy 2.x** | `numpy.core.multiarray failed to import`，YOLO 节点静默崩溃 | `/usr/bin/python3 -m pip install --user "matplotlib>=3.8"` |
| **git HTTP/2 不稳定** | `RPC failed; curl 16 Error in the HTTP2 framing layer` | clone 时加 `git -c http.version=HTTP/1.1` |
| **没有系统 pip** | `/usr/bin/python3: No module named pip` | `sudo apt install -y python3-pip` |

## 项目结构

```
ch4-yolo-ik-grasp/
├── grasp_sim/              # 仿真包 (ament_cmake)
│   ├── config/             # ros2_control / SRDF / 桥接 / kinematics
│   ├── models/             # UR5 + Robotiq 合并 URDF (xacro)
│   ├── worlds/             # SDF 场景（桌子、9 物体、内联相机）
│   ├── launch/             # sim.launch.py / full.launch.py
│   └── scripts/            # grasp_planner.py + view_camera.py
├── yolo_vision/            # YOLO 检测包 (ament_python)
│   ├── yolo_vision/        # detector_node.py
│   └── models/best.pt      # 预训练模型（YOLOv8n，单类）
├── ur5-yolo-grasp-demo.mp4 # 抓取演示视频
└── README.md
```

## 致谢

- [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [PickNik Robotics MoveIt2](https://github.com/ros-planning/moveit2)
- [ros-controls/gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
