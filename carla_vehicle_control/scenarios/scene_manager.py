import sys
import os
import carla
import yaml
import csv
import math

sys.path.append(os.path.expanduser('~/carla/PythonAPI/carla'))

# ── 修复问题二：用脚本自身位置构造默认路径，与工作目录无关 ──────────────────────
# 脚本位于: .../ros2_carla_ws/src/carla_vehicle_control/carla_vehicle_control/scenarios/
# config 位于: .../ros2_carla_ws/src/carla_vehicle_control/config/
_SCRIPT_DIR      = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR      = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', '..', 'config'))
_DEFAULT_PARAMS  = os.path.join(_CONFIG_DIR, 'vehicle_params.yaml')
_DEFAULT_CSV_DIR = _CONFIG_DIR


class SceneManager:
    def __init__(self, params_path, csv_dir):
        """
        params_path: vehicle_params.yaml 路径
        csv_dir:     存放 route_X.csv 的目录
        """
        # 统一转为绝对路径，使脚本在任意工作目录下均可运行
        params_path = os.path.abspath(params_path)
        csv_dir     = os.path.abspath(csv_dir)

        with open(params_path, 'r') as f:
            self.params = yaml.safe_load(f)

        self.csv_dir = csv_dir
        self.vehicle = None
        self.spectator = None

        # 连接 CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)

        scene_cfg   = self.params['scene']
        target_map  = scene_cfg['map']   # 例如 "Town04"

        # ── 修复问题一：只有当前地图与目标不同时才加载，避免重载同一地图导致闪退 ──
        current_map = self.client.get_world().get_map().name  # 如 "Carla/Maps/Town04"
        if target_map not in current_map:
            print(f"正在加载地图: {target_map}（当前: {current_map}）")
            self.client.load_world(target_map)
        else:
            print(f"地图已是 {target_map}，跳过重新加载")

        self.world = self.client.get_world()

        # 设置仿真步长（同步模式）
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = scene_cfg['fixed_delta_seconds']
        self.world.apply_settings(settings)

        # 设置天气
        weather = getattr(carla.WeatherParameters, scene_cfg['weather'])
        self.world.set_weather(weather)

        self.spawn_points = self.world.get_map().get_spawn_points()
        self.spectator    = self.world.get_spectator()

    def load_trajectory(self, route_id):
        """读取对应路线的 CSV，返回轨迹点列表"""
        csv_path   = os.path.join(self.csv_dir, f'route_{route_id}.csv')
        trajectory = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                trajectory.append({
                    's':     float(row['s']),
                    'x':     float(row['x']),
                    'y':     float(row['y']),
                    'yaw':   float(row['yaw']),
                    'kappa': float(row['kappa']),
                    'speed': float(row['speed']),
                })
        return trajectory

    def spawn_vehicle(self, route_id):
        """在路线起点生成车辆"""
        trajectory = self.load_trajectory(route_id)
        start_pt   = trajectory[0]

        ego_cfg           = self.params['vehicles']['ego']
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.find(ego_cfg['blueprint'])
        bp.set_attribute('color', ego_cfg['color'])
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', ego_cfg['name'])

        spawn_transform = carla.Transform(
            carla.Location(x=start_pt['x'], y=start_pt['y'], z=0.5),
            carla.Rotation(yaw=math.degrees(start_pt['yaw']))
        )

        self.vehicle = self.world.spawn_actor(bp, spawn_transform)
        print(f"车辆已生成: {ego_cfg['name']} 位于路线{route_id}起点")
        return self.vehicle

    def visualize_trajectory(self, route_id):
        """在 CARLA 场景中画出参考轨迹"""
        trajectory = self.load_trajectory(route_id)
        debug  = self.world.debug
        colors = {
            'A': carla.Color(0, 255, 0),
            'B': carla.Color(0, 100, 255),
            'C': carla.Color(255, 50, 0),
        }
        color = colors.get(route_id, carla.Color(255, 255, 255))

        for i, pt in enumerate(trajectory):
            loc = carla.Location(x=pt['x'], y=pt['y'], z=0.3)
            if i == 0:
                debug.draw_string(loc, f'[{route_id}] START', life_time=0, color=color)
                debug.draw_point(loc, size=0.3, life_time=0, color=color)
            elif i == len(trajectory) - 1:
                debug.draw_string(loc, f'[{route_id}] END', life_time=0, color=color)
                debug.draw_point(loc, size=0.3, life_time=0, color=color)
            else:
                debug.draw_point(loc, size=0.08, life_time=0, color=color)

        print(f"路线{route_id} 轨迹已可视化，共{len(trajectory)}个点")

    def set_spectator(self):
        if self.vehicle is None:
            print("车辆未生成")
            return
        transform = self.vehicle.get_transform()

        yaw_rad  = math.radians(transform.rotation.yaw)
        offset_x = -15 * math.cos(yaw_rad)
        offset_y = -15 * math.sin(yaw_rad)

        spectator_transform = carla.Transform(
            carla.Location(
                x=transform.location.x + offset_x,
                y=transform.location.y + offset_y,
                z=transform.location.z + 12
            ),
            carla.Rotation(pitch=-35, yaw=transform.rotation.yaw)
        )
        self.spectator.set_transform(spectator_transform)

    def tick(self):
        """推进一步仿真，同步模式下必须手动 tick"""
        self.world.tick()

    def update_spectator(self):
        """每帧更新摄像机跟随车辆"""
        if self.vehicle is None:
            return
        self.set_spectator()

    def destroy(self):
        """清理车辆，恢复异步模式"""
        if self.vehicle is not None:
            self.vehicle.destroy()
            print("车辆已销毁")
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--route',   type=str, default='A',
                        help='路线ID: A/B/C')
    parser.add_argument('--params',  type=str, default=_DEFAULT_PARAMS,
                        help='vehicle_params.yaml 路径（默认由脚本位置自动推导）')
    parser.add_argument('--csv_dir', type=str, default=_DEFAULT_CSV_DIR,
                        help='route_X.csv 所在目录（默认由脚本位置自动推导）')
    args = parser.parse_args()

    manager = SceneManager(args.params, args.csv_dir)
    manager.spawn_vehicle(args.route)
    manager.visualize_trajectory(args.route)
    manager.set_spectator()

    print("场景已就绪，按 Ctrl+C 退出")
    # try:
    #     while True:
    #         manager.tick()
    #         manager.update_spectator()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     manager.destroy()