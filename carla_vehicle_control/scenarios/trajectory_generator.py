import sys
import os
import carla
import csv
import math
"""
1.  切换到town04
python3 - <<'PY'
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(20.0)
client.load_world('Town04')
print('Switched to Town04')
PY

2. 生成路径
python3 /home/nor/ros2_carla_ws/src/carla_vehicle_control/carla_vehicle_control/scenarios/trajectory_generator.py --route A --save /home/nor/ros2_carla_ws/src/carla_vehicle_control/config/route_A.csv
python3 /home/nor/ros2_carla_ws/src/carla_vehicle_control/carla_vehicle_control/scenarios/trajectory_generator.py --route B --save /home/nor/ros2_carla_ws/src/carla_vehicle_control/config/route_B.csv
python3 /home/nor/ros2_carla_ws/src/carla_vehicle_control/carla_vehicle_control/scenarios/trajectory_generator.py --route C --save /home/nor/ros2_carla_ws/src/carla_vehicle_control/config/route_C.csv

"""
CURVE_SPEED_DEFAUT = 2.0
sys.path.append(os.path.expanduser('~/carla/PythonAPI/carla'))
from agents.navigation.global_route_planner import GlobalRoutePlanner

# ============================================================
# 路线定义
# segments: 每段 (起点spawn_id, 终点spawn_id, 保留比例)
# ref_speed: 该路线的参考速度 m/s
# ============================================================
ROUTES = {
    'A': {
        'description': '纯直道 约184m 纵向控制baseline',
        'spawn_start': 161,
        'segments': [(161, 167, 1.0)],
        'ref_speed': 3.0,
    },
    'B': {
        'description': '直道+弯道 约152m 横纵向综合测试',
        'spawn_start': 86,
        'segments': [(86, 161, 1.0)],
        'ref_speed': 3.0,
    },
    'C': {
        'description': '环形路段 多弯道 横向压力测试',
        'spawn_start': 340,
        'segments': [
            (340, 175, 1.0),
            (175, 274, 1.0),
            (274, 338, 0.8),
        ],
        'ref_speed': 3.0,
    },
}


class TrajectoryGenerator:
    def __init__(self, host='localhost', port=2000, sampling=0.5, use_bridge_frame=True):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.spawn_points = self.map.get_spawn_points()
        self.grp = GlobalRoutePlanner(self.map, sampling_resolution=sampling)
        # True: 输出给 ROS/bridge 使用的坐标系（y 取反，yaw 取反）
        self.use_bridge_frame = use_bridge_frame

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def generate(self, route_id):
        assert route_id in ROUTES, f"路线ID {route_id} 不存在，可选: {list(ROUTES.keys())}"
        route_cfg = ROUTES[route_id]
        all_waypoints = []

        for (start_idx, end_idx, ratio) in route_cfg['segments']:
            start_loc = self.spawn_points[start_idx].location
            end_loc = self.spawn_points[end_idx].location
            seg = self.grp.trace_route(start_loc, end_loc)
            seg = seg[:int(len(seg) * ratio)]
            all_waypoints.extend(seg)

        # 第一遍：收集原始点（同时处理坐标系和yaw归一化）
        raw = []
        for wp, _ in all_waypoints:
            t = wp.transform
            x = t.location.x
            y = t.location.y
            yaw = math.radians(t.rotation.yaw)
            yaw = self._normalize_angle(yaw)

            if self.use_bridge_frame:
                y = -y
                yaw = self._normalize_angle(-yaw)

            raw.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'speed': route_cfg['ref_speed'],
            })
        if route_id == 'B' and len(raw) > 1:
            raw = raw[1:]

        # 去掉连续重复点（避免 s 不增长、重复行）
        dedup = []
        for pt in raw:
            if not dedup:
                dedup.append(pt)
                continue
            dx = pt['x'] - dedup[-1]['x']
            dy = pt['y'] - dedup[-1]['y']
            if math.hypot(dx, dy) < 1e-4:
                continue
            dedup.append(pt)

        # 第二遍：计算 s 和 kappa
        trajectory = []
        s = 0.0
        for i, pt in enumerate(dedup):
            if i == 0:
                ds = 0.0
                kappa = 0.0
            else:
                dx = pt['x'] - dedup[i - 1]['x']
                dy = pt['y'] - dedup[i - 1]['y']
                ds = math.hypot(dx, dy)

                dyaw = self._normalize_angle(pt['yaw'] - dedup[i - 1]['yaw'])
                kappa = dyaw / ds if ds > 1e-6 else 0.0

            s += ds
            trajectory.append({
                's': round(s, 3),
                'x': round(pt['x'], 2),
                'y': round(pt['y'], 2),
                'yaw': round(self._normalize_angle(pt['yaw']), 6),
                'kappa': round(kappa, 6),
                'speed': pt['speed'],
            })

        # 第三遍：弯道减速处理
        CURVE_KAPPA_THRESHOLD = 0.05
        CURVE_SPEED = CURVE_SPEED_DEFAUT
        PREVIEW_POINTS = 20

        n = len(trajectory)
        for i in range(n):
            look_ahead_end = min(i + PREVIEW_POINTS, n)
            is_curve_ahead = any(
                abs(trajectory[j]['kappa']) > CURVE_KAPPA_THRESHOLD
                for j in range(i, look_ahead_end)
            )
            if is_curve_ahead or abs(trajectory[i]['kappa']) > CURVE_KAPPA_THRESHOLD:
                trajectory[i]['speed'] = CURVE_SPEED

        # 第四遍：终点渐停（按当前速度动态计算）
        # v_allow = sqrt(2 * a * remain)
        DECEL_COMFORT = 0.60     # m/s^2，舒适减速度（可调 0.5~0.8）
        STOP_BUFFER = 1.5        # m，额外缓冲距离
        MIN_STOP_LEN = 3.0       # m，最小减速长度
        FINAL_ZERO_POINTS = 3    # 最后几个点强制 0 速

        if len(trajectory) > 0:
            s_end = trajectory[-1]["s"]

            for pt in trajectory:
                remain = s_end - pt["s"]
                if remain <= 0.0:
                    pt["speed"] = 0.0
                    continue

                # 当前点速度决定该点“理论所需减速距离”
                v_cur = max(0.0, pt["speed"])
                need_len = (v_cur * v_cur) / (2.0 * DECEL_COMFORT + 1e-9) + STOP_BUFFER
                need_len = max(MIN_STOP_LEN, need_len)

                # 若进入减速区，则按物理上限约束速度
                if remain < need_len:
                    v_allow = math.sqrt(max(0.0, 2.0 * DECEL_COMFORT * remain))
                    pt["speed"] = round(min(pt["speed"], v_allow), 3)

            # 最后N个点强制置零，确保停住
            n = len(trajectory)
            for i in range(max(0, n - FINAL_ZERO_POINTS), n):
                trajectory[i]["speed"] = 0.0


        return trajectory

    def save_csv(self, route_id, save_path):
        """
        生成轨迹并保存为CSV
        """
        trajectory = self.generate(route_id)
        with open(save_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['s', 'x', 'y', 'yaw', 'kappa', 'speed'])
            writer.writeheader()
            writer.writerows(trajectory)
        print(f"路线{route_id} 共{len(trajectory)}个点，已保存到 {save_path}")
        return trajectory

    def visualize(self, route_id):
        """
        在CARLA场景中画出轨迹
        """
        trajectory = self.generate(route_id)
        debug = self.world.debug
        colors = {
            'A': carla.Color(0, 255, 0),
            'B': carla.Color(0, 100, 255),
            'C': carla.Color(255, 50, 0),
        }
        color = colors.get(route_id, carla.Color(255, 255, 255))

        for i, pt in enumerate(trajectory):
            draw_y = -pt['y'] if self.use_bridge_frame else pt['y']
            loc = carla.Location(x=pt['x'], y=draw_y, z=0.5)
            if i == 0:
                debug.draw_string(loc, f'[{route_id}] START', life_time=120, color=color)
                debug.draw_point(loc, size=0.3, life_time=120, color=color)
            elif i == len(trajectory) - 1:
                debug.draw_string(loc, f'[{route_id}] END', life_time=120, color=color)
                debug.draw_point(loc, size=0.3, life_time=120, color=color)
            else:
                debug.draw_point(loc, size=0.1, life_time=120, color=color)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--route', type=str, default='A', help='路线ID: A/B/C')
    parser.add_argument('--save', type=str, default='', help='保存CSV路径，不填则只可视化')
    args = parser.parse_args()

    gen = TrajectoryGenerator()
    if args.save:
        gen.save_csv(args.route, args.save)
    gen.visualize(args.route)
    print('完成')