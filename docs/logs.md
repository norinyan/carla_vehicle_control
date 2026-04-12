# 开发日志


## 项目问题记录（近期） 2026-4-6

1. 横向控制过弯晚打方向盘  
   原因：前瞻点设置过小（`lookahead_pts = 2`）。  
   处理：将前瞻点改为 `lookahead_pts = 8`，提前进入转向，过弯跟踪改善。  

2. 横向控制过弯反向打方向盘  
   原因：航向误差符号方向写反（`e_yaw = self._norm(yaw_ref - yaw)`）。  
   处理：改为带负号（`e_yaw = -self._norm(yaw_ref - yaw)`），并增加前馈项，转向方向恢复正常且过弯更稳定。  

3. 油门标定一维模型不适配多速度场景  
   原因：一维标定无法同时覆盖不同速度下的动力学差异。  
   处理：改为二维标定（油门-速度 -> 加速度），提高不同车速下的可用性。  

4. 油门标定后纵向速度存在稳态误差  
   原因：纯反馈控制在该工况下存在残余偏差。  
   处理：增加前馈补偿，减小稳态误差，纵向速度跟踪更准确。  






## 2026-03-29

### 本次工作内容

#### 1. 环境确认
- CARLA 版本：0.9.13
- 地图：Town04
- ROS2：Foxy
- Python 环境：conda carla 环境 + ROS2 Foxy 同时激活
- 工作空间：`~/ros2_carla_ws`

#### 2. 创建 ROS2 包
- 包名：`carla_vehicle_control`
- 类型：`ament_python`
- 路径：`~/ros2_carla_ws/src/carla_vehicle_control`

#### 3. 建立包目录结构
在 `carla_vehicle_control/carla_vehicle_control/` 下建立以下模块目录：
- `controllers/`：横纵向控制器
- `scenarios/`：场景管理、轨迹生成
- `data_recorder/`：数据记录
- `visualizer/`：实时可视化
- `utils/`：工具函数

包外建立：
- `config/`：存放配置文件和轨迹CSV
- `launch/`：启动文件
- `results/`：实验结果

#### 4. 路线规划与轨迹生成
使用 CARLA `GlobalRoutePlanner` 规划三条路线，采样间距 0.5m，生成 CSV 文件存于 `config/`。

CSV 字段：`s, x, y, yaw, kappa, speed`
- `s`：累计弧长（m），精度3位小数
- `x, y`：位置坐标（m），精度2位小数
- `yaw`：航向角（rad），精度6位小数
- `kappa`：曲率（1/m），精度6位小数
- `speed`：参考速度（m/s）

**路线A** `config/route_A.csv`
- 描述：纯直道，纵向控制 baseline
- spawn 点：161 → 167
- 长度：约 180m（361个点）
- 参考速度：3.0 m/s

**路线B** `config/route_B.csv`
- 描述：直道 + 弯道，横纵向综合测试
- spawn 点：86 → 161
- 长度：约 151m（303个点）
- 参考速度：3.0 m/s

**路线C** `config/route_C.csv`
- 描述：环形路段，多弯道，横向压力测试
- spawn 点：340 → 175 → 274 → 338（最后一段取80%）
- 长度：约 220m（441个点）
- 参考速度：3.0 m/s

### 遇到的问题
1. `GlobalRoutePlanner` 依赖 `networkx`，默认未安装，`pip install networkx` 解决
2. 路线C早期选点方向错误（spawn点在对向车道），通过筛选yaw角重新选点解决
3. 路线C早期用直线箭头连接spawn点导致穿墙，改用 GlobalRoutePlanner 沿道路规划解决
4. B路线起点与轨迹点之间有小段gap，属正常现象，spawn点不一定在路网采样点上，控制器用最近点匹配不受影响

### 小结
完成了项目骨架搭建和三条参考轨迹的生成，轨迹数据包含控制所需的全部字段（s, x, y, yaw, kappa, speed），覆盖横向PD、纵向PID、MPC等主流控制算法的输入需求。下一步进入场景管理和车辆生成。

---

### 当前目录结构
```tree
carla_vehicle_control/
├── carla_vehicle_control
│   ├── controllers
│   │   └── __init__.py
│   ├── data_recorder
│   │   └── __init__.py
│   ├── __init__.py
│   ├── scenarios
│   │   ├── __init__.py
│   │   └── trajectory_generator.py
│   ├── utils
│   │   └── __init__.py
│   └── visualizer
│       └── __init__.py
├── config
│   ├── route_A.csv
│   ├── route_B.csv
│   └── route_C.csv
├── launch
├── logs.md
├── package.xml
├── resource
│   └── carla_vehicle_control
├── results
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
\```
```