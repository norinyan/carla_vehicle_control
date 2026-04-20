# carla_vehicle_control

基于 CARLA + ROS 2 的轨迹跟踪。

项目目标：给定一条离线参考轨迹，控制 `ego_vehicle` 在 `Town04` 里稳定跑完，并且方便切换横向控制器、做录像、看调参结果。

输入是 `route_X.csv`，输出就是 CARLA 的油门、刹车、转向命令。

## 当前实现到哪一步

- 地图固定在 `Town04`
- 车辆固定为 `vehicle.lincoln.mkz_2020`
- 参考轨迹支持 `A / B / C` 三条路线
- 横向控制器支持 `PD / Pure Pursuit / Stanley`
- 纵向控制器目前是 `PID`
- 油门控制不是简单线性映射，而是走二维标定表
- 控制节点带有 `WAITING -> DRIVING -> STOPPING -> STOPPED` 状态机
- 有一个 matplotlib 可视化窗口，用来看速度、航向误差、控制输出和实际轨迹

## 演示

### Stanley + PID

![Stanley + PID](docs/media/st-pid.gif)

### Pure Pursuit + PID

![Pure Pursuit + PID](docs/media/pp-pid.gif)

### PD + PID

![PD + PID](docs/media/pd-pid.gif)

## 目录结构

```text
carla_vehicle_control/
├── carla_vehicle_control/
│   ├── controllers/
│   │   ├── base_controller.py
│   │   ├── lat_pd.py
│   │   ├── lat_pp.py
│   │   ├── lat_st.py
│   │   └── lon_pid.py
│   ├── scenarios/
│   │   ├── scene_manager.py
│   │   └── trajectory_generator.py
│   ├── utils/
│   │   └── calibration.py
│   ├── trajectory_tracker.py
│   └── visualizer.py
├── config/
│   ├── controller_params.yaml
│   ├── vehicle_params.yaml
│   ├── ego_vehicle.json
│   ├── route_A.csv
│   ├── route_B.csv
│   ├── route_C.csv
│   └── calibration_table.csv
├── docs/
│   ├── logs.md
│   ├── visualization_design.md
│   └── media/
├── launch/
│   ├── run_control.launch.py
│   └── spawn_scene.launch.py
└── test/
```

## 环境

- Ubuntu 20.04
- ROS 2 Foxy
- CARLA 0.9.13
- `carla_ros_bridge`
- `carla_spawn_objects`
- Python 依赖：`PyYAML`、`numpy`、`matplotlib`

如果你重新生成轨迹，还需要 CARLA PythonAPI 里的 `GlobalRoutePlanner` 依赖环境可用。

## 快速开始

先编译：

```bash
cd ~/ros_ws
colcon build --packages-select carla_vehicle_control
source install/setup.bash
```

### 1. 启动 CARLA

```bash
carla
```

如果当前不是 `Town04`，先切过去。

### 2. 启动 ros bridge 并生成 ego 车

```bash
ros2 launch carla_vehicle_control spawn_scene.launch.py route:=A
```

`route` 可选 `A`、`B`、`C`。

### 3. 启动控制节点

```bash
ros2 run carla_vehicle_control trajectory_tracker --ros-args -p route:=A
```

### 4. 启动可视化

```bash
ros2 run carla_vehicle_control visualizer route:=A
```

### 5. 发送启动信号

```bash
ros2 topic pub --once /carla_viz/start std_msgs/Bool "data: true"
```

如果要切路线，`spawn_scene`、`trajectory_tracker`、`visualizer` 三边的 `route` 要保持一致。

## 控制器切换

控制器选择在 [`config/controller_params.yaml`](config/controller_params.yaml) 里。

```yaml
trajectory_tracker:
  lat_ctrl: "lat_pd"   # lat_pd / lat_pp / lat_st
  lon_ctrl: "lon_pid"
```

当前支持：

- `lat_pd`：横向 PD，额外叠加航向误差和简单曲率前馈
- `lat_pp`：Pure Pursuit，自适应前视距离
- `lat_st`：Stanley，基于前轴误差
- `lon_pid`：纵向 PID，输出目标加速度

## 三条路线

| 路线 | 点数 | 长度 | 用途 |
| --- | ---: | ---: | --- |
| A | 355 | 179.15 m | 纯直道，适合看纵向控制 baseline |
| B | 297 | 146.27 m | 直道 + 弯道，横纵向综合测试 |
| C | 434 | 226.14 m | 环形多弯道，横向压力测试 |

轨迹 CSV 字段统一为：

```text
s, x, y, yaw, kappa, speed
```

其中：

- `s`：累计弧长
- `x, y`：参考位置
- `yaw`：参考航向角
- `kappa`：曲率
- `speed`：参考速度

## 控制主流程

`trajectory_tracker.py`：

1. 读取 `controller_params.yaml`，实例化横纵向控制器
2. 读取 `route_X.csv`
3. 订阅 `/carla/ego_vehicle/odometry_ego`
4. 在当前参考点附近做最近点搜索，并取前方 `N` 个参考点
5. 横向控制器输出 `steer`
6. 纵向控制器输出目标加速度 `a_cmd`
7. 用 `calibration_table.csv` 把 `a_cmd + 当前车速` 映射成油门
8. 负加速度区间按比例映射为刹车
9. 到终点附近进入停车状态机，完成减速和驻车

## 纵向标定

当前纵向控制不是“PID 输出直接当油门”。

项目里先做了一张二维标定表：

- 行：油门开度
- 列：速度区间
- 表值：该油门、该速度下的平均加速度

标定表文件是 [`config/calibration_table.csv`](config/calibration_table.csv)。

运行时逻辑是：

1. `lon_pid` 先算出目标加速度
2. `_interp_2d()` 根据当前车速，在标定表里反查对应油门
3. 再加一个和速度相关的小前馈补偿

这么做的原因很简单：同一个油门在不同速度下的加速度不一样，单一线性映射不够用。

## 可视化

当前版本的可视化节点会订阅 `/carla_viz/control_state`，显示：

- 实际速度和参考速度
- 航向误差
- 转向输出
- 纵向输出
- 参考轨迹和实际轨迹

这部分是为了录屏和调参，不是正式 HMI。

## 轨迹生成

参考轨迹由 [`carla_vehicle_control/scenarios/trajectory_generator.py`](carla_vehicle_control/scenarios/trajectory_generator.py) 生成。

生成过程里做了几件事：

- 用 `GlobalRoutePlanner` 沿道路网络规划
- 按 bridge 坐标系转换 `y` 和 `yaw`
- 计算 `s` 和 `kappa`
- 弯道提前降速
- 终点做渐停速度规划
