#!/usr/bin/env python3
"""CARLA 油门二维标定。

流程：
1. 采集原始数据：raw_calibration.csv（throttle, velocity, acceleration）
2. 后处理聚合：calibration_table.csv（行=throttle，列=速度bin，值=平均加速度）
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Dict, List, Tuple

import carla


# =========================
# 配置参数
# =========================
HOST = "localhost"
PORT = 2000
MAP_NAME = "Town04"
BLUEPRINT = "vehicle.lincoln.mkz_2020"

SPAWN_X = 105.29
SPAWN_Y = -170.06
SPAWN_Z = 0.6
SPAWN_YAW = 0.333

DT = 0.05
CAL_TIME = 10.0
THROTTLE_LIST = [0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.43, 0.46, 0.50, 0.60, 0.70]
SPEED_INTERVALS = [
    (0.25, 0.75, "0.5"),
    (0.75, 1.25, "1.0"),
    (1.25, 1.75, "1.5"),
    (1.75, 2.25, "2.0"),
    (2.25, 2.75, "2.5"),
    (2.75, 3.25, "3.0"),
    (3.25, 3.75, "3.5"),
    (3.75, 4.25, "4.0"),
    (4.25, 4.75, "4.5"),
    (4.75, 5.25, "5.0"),
    (5.5, 6.5, "6.0"),
    (6.5, 7.5, "7.0"),
    (7.5, 8.5, "8.0"),
    (8.5, 9.5, "9.0"),
    (9.5, 10.5, "10.0"),
]
BRAKE_K = 0.2


def clip(value: float, low: float, high: float) -> float:
    """把 value 限制在 [low, high]。"""
    return max(low, min(high, value))


def desired_acc_to_brake(a_desired: float, k: float = BRAKE_K) -> float:
    """刹车线性映射：a_des<0 时生效。"""
    if a_desired >= 0.0:
        return 0.0
    return clip(abs(a_desired) * k, 0.0, 1.0)


def load_route_start_point(route_csv: Path) -> Tuple[float, float]:
    """读取 route_A.csv，y 取反，返回首个点。"""
    with route_csv.open("r", newline="") as f:
        reader = csv.DictReader(f)
        first = next(reader, None)
    if first is None:
        raise RuntimeError(f"route_A.csv is empty: {route_csv}")
    return float(first["x"]), -float(first["y"])


def clear_world_traffic(world: carla.World, client: carla.Client) -> None:
    """清空 NPC 车辆与行人。"""
    actors = world.get_actors()
    ids: List[int] = []
    ids.extend(a.id for a in actors.filter("vehicle.*"))
    ids.extend(a.id for a in actors.filter("walker.pedestrian.*"))
    ids.extend(a.id for a in actors.filter("controller.ai.walker"))
    if ids:
        client.apply_batch_sync([carla.command.DestroyActor(x) for x in ids], True)


def build_spawn_transform(spawn_x: float, spawn_y: float) -> carla.Transform:
    """构造出生点位姿。"""
    return carla.Transform(
        carla.Location(x=spawn_x, y=spawn_y, z=SPAWN_Z),
        carla.Rotation(yaw=SPAWN_YAW),
    )


def spawn_vehicle(world: carla.World, spawn_transform: carla.Transform) -> carla.Vehicle:
    """生成标定车辆。"""
    bp = world.get_blueprint_library().find(BLUEPRINT)
    actor = world.try_spawn_actor(bp, spawn_transform)
    if actor is None:
        raise RuntimeError("Failed to spawn calibration vehicle.")
    return actor  # type: ignore[return-value]


def get_speed(vehicle: carla.Vehicle) -> float:
    """读取当前车速（m/s）。"""
    v = vehicle.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def reset_vehicle(vehicle: carla.Vehicle, world: carla.World, spawn_transform: carla.Transform) -> None:
    """传送回起点、清零速度、tick 20 帧稳定。"""
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    vehicle.set_transform(spawn_transform)
    for _ in range(20):
        world.tick()


def write_raw_header(raw_csv: Path) -> None:
    """创建/覆盖 raw_calibration.csv，并写表头。"""
    with raw_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["throttle", "velocity", "acceleration"])


def append_raw_rows(raw_csv: Path, rows: List[Tuple[float, float, float]]) -> None:
    """追加原始采样行。"""
    with raw_csv.open("a", newline="") as f:
        writer = csv.writer(f)
        for throttle, velocity, acceleration in rows:
            writer.writerow([f"{throttle:.2f}", f"{velocity:.6f}", f"{acceleration:.6f}"])


def collect_raw_data(raw_csv: Path) -> None:
    """采集每个油门下的完整时间序列。"""
    project_root = Path(__file__).resolve().parents[2]
    route_csv = project_root / "config" / "route_A.csv"
    spawn_x, spawn_y = load_route_start_point(route_csv)
    spawn_transform = build_spawn_transform(spawn_x, spawn_y)

    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    world = client.load_world(MAP_NAME)

    old_settings = world.get_settings()
    sync_settings = world.get_settings()
    sync_settings.synchronous_mode = True
    sync_settings.fixed_delta_seconds = DT
    # 地图加载后先 tick，避免在异步加载未完成时设置同步模式
    world.tick()
    world.tick()
    world.apply_settings(sync_settings)
    print(world.get_settings().synchronous_mode)  # 应该输出 True

    tm = client.get_trafficmanager()
    tm.set_synchronous_mode(True)

    vehicle: carla.Vehicle | None = None
    try:
        world.set_weather(carla.WeatherParameters.ClearNoon)
        clear_world_traffic(world, client)
        for _ in range(3):
            world.tick()

        vehicle = spawn_vehicle(world, spawn_transform)
        # 刚 spawn 后先 tick，让车辆物理状态完成初始化
        for _ in range(5):
            world.tick()
        steps = int(CAL_TIME / DT)
        write_raw_header(raw_csv)

        for throttle in THROTTLE_LIST:
            reset_vehicle(vehicle, world, spawn_transform)
            rows: List[Tuple[float, float, float]] = []
            velocity_window: List[float] = []

            for _ in range(steps):
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=0.0, brake=0.0))
                world.tick()

                v_now = get_speed(vehicle)
                # 使用 5 帧滑动窗口减小差分噪声：a = (v[-1] - v[0]) / (4 * DT)
                velocity_window.append(v_now)
                if len(velocity_window) > 5:
                    velocity_window.pop(0)

                if len(velocity_window) < 5:
                    a_now = 0.0
                else:
                    a_now = (velocity_window[-1] - velocity_window[0]) / (4.0 * DT)
                rows.append((throttle, v_now, a_now))

            append_raw_rows(raw_csv, rows)
            print(f"[Raw] throttle={throttle:.2f}, samples={len(rows)}")

    finally:
        if vehicle is not None and vehicle.is_alive:
            vehicle.destroy()
        tm.set_synchronous_mode(False)
        world.apply_settings(old_settings)


def build_2d_table(raw_csv: Path, table_csv: Path) -> None:
    """将 raw_calibration.csv 聚合为二维标定表。"""
    groups: Dict[Tuple[float, str], List[float]] = {}
    with raw_csv.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            throttle = float(row["throttle"])
            velocity = float(row["velocity"])
            accel = float(row["acceleration"])
            interval_label: str | None = None
            for v_low, v_high, label in SPEED_INTERVALS:
                if v_low <= velocity < v_high:
                    interval_label = label
                    break
            if interval_label is None:
                continue
            key = (throttle, interval_label)
            groups.setdefault(key, []).append(accel)

    headers = ["throttle"] + [label for _, _, label in SPEED_INTERVALS]
    with table_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)

        for throttle in THROTTLE_LIST:
            row = [f"{throttle:.2f}"]
            for _, _, label in SPEED_INTERVALS:
                samples = groups.get((throttle, label), [])
                if len(samples) < 3:
                    row.append("NaN")
                else:
                    avg_a = sum(samples) / len(samples)
                    row.append(f"{avg_a:.6f}")
            writer.writerow(row)


def read_table_for_lookup(table_csv: Path) -> Tuple[List[float], List[float], List[List[float | None]]]:
    """读取二维标定表（NaN -> None）。"""
    with table_csv.open("r", newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"Empty calibration table: {table_csv}")

    speed_bins = [float(x) for x in rows[0][1:]]
    throttles: List[float] = []
    grid: List[List[float | None]] = []
    for r in rows[1:]:
        throttles.append(float(r[0]))
        vals: List[float | None] = []
        for x in r[1:]:
            if x == "NaN":
                vals.append(None)
            else:
                vals.append(float(x))
        grid.append(vals)
    return throttles, speed_bins, grid


def linear_interpolate(x0: float, y0: float, x1: float, y1: float, x: float) -> float:
    """一维线性插值。"""
    if abs(x1 - x0) < 1e-9:
        return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


def throttle_from_table(v_now: float, a_des: float, table_csv: Path) -> float:
    """MPC 前馈查表：输入 (v_now, a_des) 输出 throttle。"""
    throttles, speed_bins, grid = read_table_for_lookup(table_csv)

    # 1) 速度列插值：每个 throttle 在 v_now 处得到一个 a(v_now)
    a_at_v: List[float | None] = []
    for i in range(len(throttles)):
        row = grid[i]
        valid = [(speed_bins[j], row[j]) for j in range(len(speed_bins)) if row[j] is not None]
        if len(valid) < 2:
            a_at_v.append(None)
            continue

        if v_now <= valid[0][0]:
            a_at_v.append(float(valid[0][1]))
            continue
        if v_now >= valid[-1][0]:
            a_at_v.append(float(valid[-1][1]))
            continue

        val: float | None = None
        for k in range(len(valid) - 1):
            x0, y0 = valid[k]
            x1, y1 = valid[k + 1]
            if x0 <= v_now <= x1:
                val = linear_interpolate(float(x0), float(y0), float(x1), float(y1), v_now)
                break
        a_at_v.append(val)

    # 2) 反查 throttle：在 a_at_v-throttle 曲线上按 a_des 插值
    pairs = [(a_at_v[i], throttles[i]) for i in range(len(throttles)) if a_at_v[i] is not None]
    if len(pairs) < 2:
        return 0.0

    pairs.sort(key=lambda x: float(x[0]))
    if a_des <= float(pairs[0][0]):
        return float(pairs[0][1])
    if a_des >= float(pairs[-1][0]):
        return float(pairs[-1][1])

    for i in range(len(pairs) - 1):
        a0, t0 = float(pairs[i][0]), float(pairs[i][1])
        a1, t1 = float(pairs[i + 1][0]), float(pairs[i + 1][1])
        if a0 <= a_des <= a1:
            return clip(linear_interpolate(a0, t0, a1, t1, a_des), 0.0, 1.0)
    return 0.0


def run_calibration() -> None:
    """执行完整流程：采集原始数据 + 生成二维表。"""
    project_root = Path(__file__).resolve().parents[2]
    raw_csv = project_root / "config" / "raw_calibration.csv"
    table_csv = project_root / "config" / "calibration_table.csv"

    collect_raw_data(raw_csv)
    build_2d_table(raw_csv, table_csv)
    print(f"[Done] raw={raw_csv}")
    print(f"[Done] table={table_csv}")


if __name__ == "__main__":
    run_calibration()
