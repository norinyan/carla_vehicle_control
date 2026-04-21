import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
from datetime import datetime
from pathlib import Path


class Visualizer(Node):

    def __init__(self, ref_path_x, ref_path_y, route):
        super().__init__("visualizer")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            Float32MultiArray,
            "/carla_viz/control_state",
            self._on_data,
            10,
        )

        # 参考路径（给小地图用）
        self.ref_path_x = ref_path_x
        self.ref_path_y = ref_path_y
        self.route = route

        # 数据缓冲：按需求保留全部帧
        self.t         = []
        self.speed_now = []
        self.speed_ref = []
        self.yaw_err   = []
        self.steer     = []
        self.accel     = []
        self.traj_x    = []
        self.traj_y    = []

        self.frame = 0
        self.latest = None

        # 记录到 data/*.csv
        data_dir = Path("/home/nor/ros2_carla_ws/src/carla_vehicle_control/data")
        data_dir.mkdir(parents=True, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = data_dir / f"visualizer_route_{self.route}_{ts}.csv"
        self.csv_fp = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_fp)
        self.csv_writer.writerow([
            "t",
            "speed_now",
            "speed_ref",
            "yaw_now",
            "yaw_ref",
            "yaw_err_deg",
            "steer",
            "accel",
            "traj_x",
            "traj_y",
        ])
        self.csv_fp.flush()
        self.get_logger().info(f"recording csv -> {self.csv_path}")

    def _on_data(self, msg):

        print(f"收到数据: {len(msg.data)} 个字段")  # 加这行
        d = msg.data
        self.frame += 1
        t = self.frame * 0.1  # 10Hz

        self.t.append(t)
        self.speed_now.append(d[0])
        self.speed_ref.append(d[1])

        err = d[3] - d[2]
        err = math.atan2(math.sin(err), math.cos(err))  # 归一化到[-pi, pi]
        self.yaw_err.append(math.degrees(err))

        self.steer.append(d[4])
        self.accel.append(d[5])
        self.traj_x.append(d[6])
        self.traj_y.append(d[7])

        self.csv_writer.writerow([
            t,
            d[0],
            d[1],
            d[2],
            d[3],
            self.yaw_err[-1],
            d[4],
            d[5],
            d[6],
            d[7],
        ])
        self.csv_fp.flush()

    def close_recorder(self):
        if hasattr(self, "csv_fp") and self.csv_fp and not self.csv_fp.closed:
            self.csv_fp.flush()
            self.csv_fp.close()



def build_figure():
    fig = plt.figure(figsize=(14, 8))
    fig.patch.set_facecolor("#1e1e2e")

    # 布局：左列4个数据图，右列1个大地图
    ax_speed  = fig.add_subplot(4, 2, 1)
    ax_yaw    = fig.add_subplot(4, 2, 3)
    ax_steer  = fig.add_subplot(4, 2, 5)
    ax_accel  = fig.add_subplot(4, 2, 7)
    ax_map    = fig.add_subplot(1, 2, 2)

    for ax in [ax_speed, ax_yaw, ax_steer, ax_accel, ax_map]:
        ax.set_facecolor("#2a2a3e")
        ax.tick_params(colors="white", labelsize=7)
        for spine in ax.spines.values():
            spine.set_edgecolor("#555577")

    ax_speed.set_title("Speed (m/s)",  color="white", fontsize=9)
    ax_yaw.set_title("Yaw Error (deg)", color="white", fontsize=9)
    ax_steer.set_title("Steer Output", color="white", fontsize=9)
    ax_accel.set_title("Accel Output (m/s²)", color="white", fontsize=9)
    ax_map.set_title("Trajectory Map", color="white", fontsize=9)
    ax_map.set_aspect("equal")

    fig.tight_layout(pad=2.0)
    return fig, ax_speed, ax_yaw, ax_steer, ax_accel, ax_map


def main(args=None):
    rclpy.init(args=args)

    # 读参考路径（给小地图画底图用）
    import os
    from ament_index_python.packages import get_package_share_directory

    # 从参数或环境变量读 route，默认 A
    import sys
    route = "A"
    for arg in sys.argv:
        if arg.startswith("route:="):
            route = arg.split(":=")[1].upper()

    pkg_share = get_package_share_directory("carla_vehicle_control")
    csv_path = os.path.join(pkg_share, "config", f"route_{route}.csv")
    ref_x, ref_y = [], []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ref_x.append(float(row["x"]))
            ref_y.append(float(row["y"]))

    node = Visualizer(ref_x, ref_y, route)

    fig, ax_speed, ax_yaw, ax_steer, ax_accel, ax_map = build_figure()

    def update(_):
        rclpy.spin_once(node, timeout_sec=0)

        t          = node.t
        speed_now  = node.speed_now
        speed_ref  = node.speed_ref
        yaw_err    = node.yaw_err
        steer      = node.steer
        accel      = node.accel
        traj_x     = node.traj_x
        traj_y     = node.traj_y

        if len(t) == 0:
            return

        # 速度
        ax_speed.cla()
        ax_speed.set_facecolor("#2a2a3e")
        ax_speed.set_title("Speed (m/s)", color="white", fontsize=9)
        ax_speed.plot(t, speed_ref, "--", color="#888888", linewidth=1, label="ref")
        ax_speed.plot(t, speed_now, color="#4fc3f7", linewidth=1.2, label="now")
        ax_speed.tick_params(colors="white", labelsize=7)
        ax_speed.legend(fontsize=7,
                facecolor="#1e1e2e", edgecolor="none")

        # 航向误差
        ax_yaw.cla()
        ax_yaw.set_facecolor("#2a2a3e")
        ax_yaw.set_title("Yaw Error (deg)", color="white", fontsize=9)
        ax_yaw.plot(t, yaw_err, color="#ffb74d", linewidth=1.2)
        ax_yaw.axhline(0, color="#555577", linewidth=0.8, linestyle="--")
        ax_yaw.tick_params(colors="white", labelsize=7)

        # 转向输出
        ax_steer.cla()
        ax_steer.set_facecolor("#2a2a3e")
        ax_steer.set_title("Steer Output [-1,1]", color="white", fontsize=9)
        ax_steer.plot(t, steer, color="#ce93d8", linewidth=1.2)
        ax_steer.axhline(0, color="#555577", linewidth=0.8, linestyle="--")
        ax_steer.set_ylim(-1.1, 1.1)
        ax_steer.tick_params(colors="white", labelsize=7)

        # 纵向输出
        ax_accel.cla()
        ax_accel.set_facecolor("#2a2a3e")
        ax_accel.set_title("Accel Output (m/s²)", color="white", fontsize=9)
        ax_accel.plot(t, accel, color="#80cbc4", linewidth=1.2)
        ax_accel.axhline(0, color="#555577", linewidth=0.8, linestyle="--")
        ax_accel.tick_params(colors="white", labelsize=7)

        # 小地图
        ax_map.cla()
        ax_map.set_facecolor("#2a2a3e")
        ax_map.set_title("Trajectory Map", color="white", fontsize=9)
        ax_map.set_aspect("equal")
        ax_map.plot(ref_x, ref_y, "--", color="#ff9800",  # 改成橙色
            linewidth=1, label="ref path")
        if len(traj_x) > 1:
            ax_map.plot(traj_x, traj_y, color="#4fc3f7",
                        linewidth=1.2, label="actual")
        if len(traj_x) > 0:
            ax_map.plot(traj_x[-1], traj_y[-1], "o",
                        color="#ff5252", markersize=6, label="vehicle")
        ax_map.tick_params(colors="white", labelsize=7)
        ax_map.legend(fontsize=7,
                    facecolor="#1e1e2e", edgecolor="none")
        fig.canvas.draw_idle()

    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()

    node.close_recorder()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
