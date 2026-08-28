# Stretch 3 仿真环境

[English](README.md) · [简体中文](README.zh-CN.md)

[![Python 3.12](https://img.shields.io/badge/Python-3.12-blue?style=flat-square)]()
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2-green?style=flat-square)]()
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-orange?style=flat-square)]()
[![Linux](https://img.shields.io/badge/Platform-Linux-lightgrey?style=flat-square)]()

基于 MuJoCo 的 [Hello Robot Stretch 3](https://hello-robot.com/stretch-3) 仿真栈，面向移动操作研究与 sim-to-real 迁移。该环境将物理仿真、ROS 2 控制接口、Nav2 导航栈、经典（IK / PID / MPC）与学习（PPO / SAC）机械臂控制器，以及可组合的 YAML 动作系统整合在一起，用于厨房抓取场景。

**亮点**

- **2D 激光 + Nav2** — 由 360 条射线组成的测距传感器发布 `sensor_msgs/LaserScan`，接入 Nav2 代价地图，配 DWB 局部规划器和 NavFn 全局规划器（无需 AMCL / map_server）
- **多种操作控制器** — 解析 IK、位置 PID、随机打靶 MPC，以及两阶段 PPO/SAC 策略，均通过同一套宏动作接口调用
- **RL 训练框架** — 独立的 Gymnasium 环境（`StretchPickEnv`），用 stable-baselines3 的 PPO 或 SAC 训练，再通过 ROS 2 部署
- **归一化 `0–1` 动作空间**，便于复现评估与强化学习

---

## 系统概览

**场景** — 一个厨房工作区，包含三张桌子、砧板、刀具、餐盘，以及可供 Stretch 夹爪抓取的食材（生菜、洋葱、番茄）。

**控制栈** — 从规则式到学习式的三层可替换方案：

| 层次 | 方法 | 相关文件 |
|------|------|----------|
| 导航 | 面向命名锚点的内置比例控制器，**或** Nav2（全局 + 局部规划器） | `src/stretch_sim/navigation.py`、`launch/nav2_sim.launch.py` |
| 操作 | IK 求解器、位置 PID、随机打靶 MPC、两阶段 PPO/SAC | `src/stretch_sim/ik.py`、`scripts/pid_arm_control.py`、`scripts/mpc_arm_control.py`、`scripts/rl_inference_arm.py` |
| 任务组合 | YAML 微动作 / 宏动作 | `config/actions.yaml` |

---

## 快速开始

### 环境要求

- Linux
- [Conda](https://docs.conda.io/en/latest/miniconda.html)
- Python 3.12
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)（ROS 工作流所需）
- [Nav2](https://docs.nav2.org/)（可选，用于 `make nav2`）：`sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`

### 安装

在仓库根目录执行：

```bash
conda env create -f environment/environment_ros2.yml
conda activate simenv_ros2
source /opt/ros/jazzy/setup.bash

make install   # pip install -e . + CLI 入口
make smoke     # 快速自动检查（无需 ROS）
make verify    # 完整环境检查
```

仅使用 MuJoCo（不启用 ROS 2）：`conda env create -f environment/environment.yml` → 环境名为 `simenv`。

RL 训练需要 conda 配置之外额外的包：

```bash
pip install gymnasium stable-baselines3 tensorboard tqdm rich
```

### 运行

**终端 1 — 仿真**

```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash
make sim
```

**终端 2 — 交互式控制器**

```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash
make controller
```

**终端 3 — Nav2 导航**（可选）

```bash
conda activate simenv_ros2 && source /opt/ros/jazzy/setup.bash
make nav2
```

需要 ROS 2 的命令（`sim`、`nav2`、`controller`、`test`）在未 source ROS 2 时会以清晰提示退出。

| 命令 | 用途 |
|------|------|
| `make install` | `pip install -e .`（可编辑安装 + CLI 工具） |
| `make smoke` | 快速自动测试（路径、MuJoCo 模型、ROS 导入） |
| `make verify` | 完整环境校验 |
| `make sim` | MuJoCo + ROS 2 仿真节点 |
| `make nav2` | 启动 Nav2 导航栈（在 `make sim` 之后运行） |
| `make controller` | 交互式 CLI |
| `make view` | MuJoCo 查看器（仅场景） |
| `make test` | ROS 2 通信测试 |

`make install` 后可用的 CLI 别名：`stretch-sim`、`stretch-controller`、`stretch-verify`。

---

## 交互式控制器

```text
stretch> help
stretch> go_to_anchor anchor=A
stretch> elevate_arm height=0.5
stretch> extend_arm length=0.8
stretch> turn_towards anchor=ORIGIN
stretch> close_gripper
stretch> rl_get_tomato
```

参数统一归一化到 **`0–1`**（`0.5` = 中位值）。CLI 支持 Tab 补全与命令历史。

### 核心动作

| 类别 | 动作 |
|------|------|
| **导航** | `go_to_anchor`、`turn_towards`、`go_to_position` |
| **机械臂** | `reset_arm`、`elevate_arm`、`extend_arm`、`rotate_wrist`、`open_gripper`、`close_gripper`、`set_gripper` |
| **经典控制** | `pid_pick`、`mpc_pick` |
| **RL 控制** | `rl_approach`、`rl_grasp_lift`、`rl_pick_object`（旧版） |
| **工具** | `wait`、`wait_for_arm` |

完整的动作定义与宏动作组合：见 [config/actions.yaml](config/actions.yaml)。

### 导航锚点

路径点在 `models/table_world.xml` 中定义为 MuJoCo site：

| 锚点 | 位置 (m) | 用途 |
|------|----------|------|
| `A` | `-0.65, 1.1` | 桌子 1，左侧 |
| `B` | ` 0.65, 1.1` | 桌子 1，右侧 |
| `C` | `-0.65, 3.6` | 桌子 2，左侧 |
| `D` | ` 0.65, 3.6` | 桌子 2，右侧 |
| `G` | ` 0.5, 3.75` | 桌子 2，前方（机械臂控制的预抓取路径点） |
| `E` | ` 1.6, 2.9` | 桌子 3，上方 |
| `F` | ` 1.6, 1.6` | 桌子 3，下方 |
| `ORIGIN` | ` 0.53, 2.32` | 原点 / 参考位置 |

---

## 导航

### 内置锚点导航

仿真节点（`scripts/stretch_ros2_sim.py`）运行一个比例控制器，将底盘驱向命名锚点或归一化的 `(x, y)` 位置。不涉及地图或规划器——它是直接基于里程计反馈的控制。

### Nav2 导航栈

为支持路径规划与避障，模型内置了 **2D 激光雷达**（360 条测距射线，以 1° 分辨率扫描水平面，以 `sensor_msgs/LaserScan` 发布到 `/scan`）。`make nav2` 会启动：

- **全局代价地图** — 15×15 m 滚动窗口，分辨率 0.05 m
- **局部代价地图** — 4×4 m 滚动窗口，DWB 局部规划器
- **全局规划器** — NavFn
- **TF 树** — `map → odom`（静态恒等）`→ base_link`（来自里程计）`→ laser`（来自 URDF，经 `robot_state_publisher`）
- **`robot_state_publisher`** — 发布 `/robot_description` 以及来自 [models/stretch_description.urdf](models/stretch_description.urdf) 的 `base_link → laser` 变换
- **RViz** — 打开 [config/nav2_view.rviz](config/nav2_view.rviz)，显示机器人模型、激光、代价地图与路径

没有 AMCL 或 `map_server`：仿真器发布真实里程计，恒等 `map → odom` 变换将其锚定在全局坐标系中。Nav2 的 `cmd_vel`（单位 m/s 与 rad/s）被重映射到 `/stretch/cmd_vel`，仿真器再根据实测的底盘标定（前进 ≈ 0.25 m/s，转向 ≈ 0.50 rad/s）将其转换为电机指令。

**在 RViz 中设置目标点** — 使用 **"2D Goal Pose"** 工具（`nav2_rviz_plugins/GoalTool`）或 **"Navigation 2"** 面板点击目标位姿；Nav2 栈通过 `/navigate_to_pose` 动作接收。同样的目标也可从命令行发送：

```bash
# 向 Nav2 导航栈发送目标点
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

配置：见 [config/nav2_params.yaml](config/nav2_params.yaml) · [launch/nav2_sim.launch.py](launch/nav2_sim.launch.py)。

---

## 操作控制器

四种可互换的机械臂控制器用于接近并抓取目标物体。每种都通过微动作（`pid_pick`、`mpc_pick`、`rl_approach` + `rl_grasp_lift`）或宏动作（`PID_get_tomato`、`MPC_get_tomato`、`rl_get_tomato`）调用。

| 控制器 | 方法 | 说明 |
|--------|------|------|
| **IK** | 解析逆运动学（`src/stretch_sim/ik.py`） | 脚本化运动，无优化 |
| **PID** | 对升降、臂伸展、腕部偏航做位置 PID | 5 阶段流程：下降 → 伸展 → 抓取 → 抬升 |
| **MPC** | 随机打靶 MPC（`scripts/mpc_arm_control.py`） | 基于运动学模型，在预测时域 H 上采样 K 组动作序列 |
| **RL** | 两阶段 PPO/SAC（`scripts/rl_inference_arm.py`） | 阶段 1 接近物体，阶段 2 抓取并抬升 |

PID 和 MPC 控制器以 ROS 2 节点形式运行，由宏动作系统通过子进程启动；它们订阅 `/stretch/joint_states`，发布到 `/stretch/joint_command`。

---

## RL 训练

独立的 MuJoCo 环境（`src/stretch_sim/rl_env.py`，`StretchPickEnv`）**无需 ROS 2 介入**即可训练机械臂，之后策略通过宏动作系统部署。

```bash
# 阶段 1 — 将末端执行器移到目标附近
python scripts/train_rl_arm.py --stage approach --target tomato1 --algo ppo --timesteps 300000

# 阶段 2 — 闭合夹爪并抬升
python scripts/train_rl_arm.py --stage grasp --target tomato1 --algo ppo --timesteps 300000

# 离线策略替代方案（样本效率更高）
python scripts/train_rl_arm.py --stage approach --target tomato1 --algo sac --timesteps 300000
```

用 TensorBoard 监控训练：

```bash
tensorboard --logdir models/tensorboard
```

检查点写入 `models/`（`<algo>_<stage>_<target>.zip`，例如 `rl_approach_tomato1.zip`），并同时保存 VecNormalize 统计文件。仓库中自带 `tomato1` 的预训练检查点。可直接用 `scripts/rl_inference_arm.py` 运行训练好的策略，或让 `rl_get_tomato` 自动依次执行两个阶段。

---

## ROS 2 接口

仿真节点：`scripts/stretch_ros2_sim.py`。

### 订阅

| 话题 | 类型 | 描述 |
|------|------|------|
| `/stretch/cmd_vel` | `geometry_msgs/Twist` | 底盘速度 |
| `/stretch/joint_command` | `std_msgs/Float64MultiArray` | 关节目标（交互式 CLI） |
| `/stretch/joint_commands` | `std_msgs/Float64MultiArray` | 关节目标（键盘 / 外部） |
| `/stretch/navigate_to_anchor` | `std_msgs/String` | 锚点：`A`–`G` 或 `ORIGIN` |
| `/stretch/turn_towards_anchor` | `std_msgs/String` | 转向某个锚点 |
| `/stretch/navigate_to_position` | `std_msgs/Float64MultiArray` | `[x, y, direction]`，`0–1` 归一化 |
| `/stretch/reset_arm` | `std_msgs/String` | 复位机械臂姿态 |

### 发布

| 话题 | 类型 | 描述 |
|------|------|------|
| `/stretch/joint_states` | `sensor_msgs/JointState` | 当前关节状态 |
| `/stretch/navigation_active` | `std_msgs/Bool` | 导航进行中为 `true` |
| `/stretch/camera/image_raw` | `sensor_msgs/Image` | 车载相机（BGR8） |
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光雷达（360 条射线） |
| `/odom` | `nav_msgs/Odometry` | 真实里程计 |

### TF 树

```
map ──(静态恒等)──> odom ──(由 qpos 动态生成)──> base_link ──(robot_state_publisher)──> laser
```

`base_link → laser` 从简化 URDF [models/stretch_description.urdf](models/stretch_description.urdf) 读取，由 `robot_state_publisher`（随 `make nav2` 启动）发布，因此激光坐标系在 RViz 和代价地图中始终可用，无需仿真器硬编码。

---

## 动作系统

动作定义在 `config/actions.yaml` 中。**微动作**是原始命令（导航、机械臂运动、PID/MPC/RL 控制、等待）；**宏动作**将其组合为序列。

| 宏动作 | 描述 |
|--------|------|
| `get_tomato` | 脚本化导航 + 夹爪抓取 |
| `rl_get_tomato` | 导航 + 两阶段 PPO 抓取 |
| `PID_get_tomato` | 导航 + 5 阶段 PID 抓取 |
| `MPC_get_tomato` | 导航 + 打靶 MPC 抓取 |
| `get_lettuce` / `get_onion` / `get_plate` / `go_to_knife` | 导航到指定目标（操作部分待实现） |
| `deliver` | 导航到配送站 |

---

## 仓库结构

```text
Stretch3_Simulation/
├── Makefile                     # make sim, nav2, controller, smoke, …
├── pyproject.toml               # pip install -e . + CLI 入口
├── config/
│   ├── actions.yaml             # 微动作 & 宏动作定义
│   ├── nav2_params.yaml         # Nav2 控制器/规划器/代价地图参数
│   └── nav2_view.rviz           # RViz 布局（机器人模型、激光、代价地图、目标点工具）
├── launch/nav2_sim.launch.py    # Nav2 导航栈（Nav2 + robot_state_publisher + RViz）
├── models/                      # MuJoCo 场景 + RL 检查点 + TensorBoard 日志
│   └── stretch_description.urdf # RViz 用简化机器人模型（robot_state_publisher）
├── src/stretch_sim/             # 库：导航、IK、RL 环境、锚点、路径
├── scripts/                     # 仿真节点、CLI、PID/MPC/RL 控制器、训练脚本
├── tools/                       # 模型工具（gen_laser.py、网格工具等）
├── environment/                 # Conda 配置（simenv_ros2、simenv）
├── docs/                        # 安装与使用指南
├── tests/                       # ROS 2 通信测试
├── assets/                      # 机器人网格
├── meshes/                      # 物体与凸碰撞网格
└── textures/
```

---

## 文档与链接

| 资源 | |
|------|---|
| [安装指南](docs/SETUP.md) | 环境与 ROS 2 安装 |
| [使用指南](docs/USAGE.md) | 控制器、导航、机械臂控制 |
| [动作定义](config/actions.yaml) | 参数与宏动作示例 |
| [Hello Robot Stretch 3](https://docs.hello-robot.com/) | 硬件文档 |
| [MuJoCo](https://mujoco.readthedocs.io/) | 物理引擎 |
| [Nav2](https://docs.nav2.org/) | 导航栈 |
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) | 中间件 |
