# Intent-MPC Docker 部署指南

## 环境要求

- Ubuntu 22.04（或其他支持 Docker 的 Linux）
- Docker 已安装
- X11 显示（用于 Gazebo 和 RViz GUI）

## 快速开始

### 1. 配置 Docker 国内镜像（如需）

```bash
echo '{"registry-mirrors":["https://docker.1ms.run","https://docker.xuanyuan.me","https://docker.m.daocloud.io"]}' | sudo tee /etc/docker/daemon.json
sudo systemctl restart docker
```

### 2. 构建镜像

```bash
cd ~/Intent-MPC
docker build -t intent-mpc .
```

首次构建约需 20-30 分钟（下载 ROS Noetic 基础镜像 + 编译项目）。

### 3. 启动容器

```bash
xhost +local:docker

docker run -it --rm \
  --name intent-mpc \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  intent-mpc
```

### 4. 运行 Demo

容器内终端1 — 启动仿真器：

```bash
source ~/.bashrc && roslaunch uav_simulator start.launch
```

新开终端2：

```bash
docker exec -it intent-mpc bash
source ~/.bashrc && roslaunch autonomous_flight intent_mpc_demo.launch
```

## 相对原始项目的修复

本 Docker 版本修复了原始代码中的以下问题：

### 1. OSQP 求解器除零（mpcPlanner.cpp）

`castMPCToQPConstraintVectors` 和 `castMPCToQPConstraintMatrix` 中，当障碍物尺寸 `osize` 为零时，`pow(osize, 2)` 作为分母导致 NaN 传播到整个 QP 问题。修复：对 osize 各分量设置 0.01 下限。

### 2. QP 解 NaN 检查（mpcPlanner.cpp）

OSQP 求解器在问题不可行时可能返回含 NaN 的解。修复：检测到 NaN 时丢弃解并重置求解器状态。

### 3. QP 输入矩阵 NaN 检查（mpcPlanner.cpp）

在求解前检查 gradient、lowerBound、upperBound 是否含 NaN，避免将无效问题送入求解器。

### 4. 零位置跳过（mpcPlanner.cpp）

当 `currPos_` 为 (0,0,0)（状态尚未更新）时跳过求解，避免构建无效 QP 问题。

### 5. Bspline 轨迹优化除零（bsplineTraj.cpp）

- `getDynamicObstacleCost`：`diff.norm()` 为零时跳过，避免除零产生 NaN
- `assignGuidePointsSemiCircle`：guideDirection 计算的除零保护
- `costFunction`：入口增加 controlPoints 和 guidePoints 大小一致性检查

### 6. Gazebo 插件 NaN 保护（quadcopterPlugin.cpp）

当力或力矩包含 NaN 时归零，防止 Gazebo 物理引擎重置无人机模型。

### 7. Tracking Controller NaN 保护（trackingController.cpp）

当加速度指令包含 NaN 时归零（等效悬停），防止无人机失控。

## 键盘控制

`start.launch` 会启动一个 Qt 键盘控制窗口：

| 键  | 功能      |
| --- | --------- |
| Z   | 起飞      |
| X   | 降落      |
| H   | 悬停      |
| W/S | 上升/下降 |
| A/D | 左转/右转 |
| I/K | 前进/后退 |
| J/L | 左移/右移 |

注意：Intent-MPC demo 模式下自主导航会接管控制。如需手动飞行，只启动 `start.launch` 即可。

## 常见问题

### Docker Hub 连接超时

配置国内镜像源，见上方步骤 1。

### Gazebo 窗口打不开

确保已执行 `xhost +local:docker`，并且 DISPLAY 环境变量正确。

### 无人机不稳定/掉落

Docker 中 Gazebo 没有 GPU 加速，仿真实时因子可能低于 1.0，导致控制不稳定。可尝试：

```bash
docker run -it --rm --name intent-mpc \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged intent-mpc
```
