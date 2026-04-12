
"""
终端1：
carla

终端2：
launch_scene route:=A
launch_scene route:=B
launch_scene route:=C

终端3：
ros2 run carla_vehicle_control trajectory_tracker --ros-args -p route:=A
ros2 run carla_vehicle_control trajectory_tracker --ros-args -p route:=B
ros2 run carla_vehicle_control trajectory_tracker --ros-args -p route:=C



"""
import os
import csv
import yaml
import math

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry

from carla_vehicle_control.controllers.lat_pd import LatPD
from carla_vehicle_control.controllers.lon_pid import LonPID


class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__("trajectory_tracker")
        
        # load config yaml 
        pkg_share = get_package_share_directory("carla_vehicle_control")
        params_path = os.path.join(pkg_share, "config", "controller_params.yaml")

        with open(params_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        # init Qos
        sensor_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT, 
            history = HistoryPolicy.KEEP_LAST, 
            depth = 10, 
        )

        # publisher
        self.pub_cmd = self.create_publisher(
            CarlaEgoVehicleControl, 
            "/carla/ego_vehicle/vehicle_control_cmd",
            10
        )

        # subscriber
        self.sub_odom = self.create_subscription(
            Odometry, 
            "/carla/ego_vehicle/odometry_ego", 
            self._on_odom, 
            sensor_qos
        )

        # timer
        self.ctrl_period = 0.1
        self.timer = self.create_timer(self.ctrl_period, self._on_timer)

        # build controller
        self.lat_ctrl = LatPD(cfg["lat_pd"])
        self.lon_ctrl = LonPID(cfg["lon_pid"])

        # states 
        self.now_state = {
            "x" : 0.0,
            "y" : 0.0,
            "speed" : 0.0,
            "yaw" : 0.0,
        }

        # reference
        self.ref_traj = []
        self.idx_ref = 0

        # 
        self.declare_parameter("route", "A")
        self.route = str(self.get_parameter("route").value).upper()
        self._load_ref_traj()

        self._load_calibration_table()

        # FSM 状态机
        self.odom_ready = False
        self.STATE_DRIVING = "DRIVING"
        self.STATE_STOPPING = "STOPPING"
        self.STATE_STOPPED = "STOPPED"
        self.tracker_state = self.STATE_DRIVING
        self.stop_distance = 2.0
        self.stop_speed = 0.2
        self.stop_brake_hold = 0.30       # 停稳保持刹车
        self.stop_brake_gain = 0.06       # 渐增刹车增益
        self.stop_integral = 0.0  

        self.BRAKE_K = 0.2

        self.get_logger().info(f"route={self.route}, points={len(self.ref_traj)}, dt={self.ctrl_period}")
    
    def _on_timer(self):
        # safety check
        if len(self.ref_traj) == 0:
            return

        # state check
        if not self.odom_ready:
            self.get_logger().warn("waiting odom...", throttle_duration_sec=2.0)
            return

        if self.tracker_state == self.STATE_DRIVING:
            self.get_logger().info("now_state: DRIVING ", once=True)
            if self._is_ready_to_stop():
                self.tracker_state = self.STATE_STOPPING
                self.stop_integral = 0.0
                self.get_logger().info("state: DRIVING -> STOPPING", once=True)
                
        if self.tracker_state == self.STATE_STOPPING:
            v = self.now_state["speed"]

            if v < self.stop_speed:
                self.tracker_state = self.STATE_STOPPED
                self.get_logger().info("state: STOPPING -> STOPPED")
                self._pub_cmd({"throttle": 0.0, "brake": self.stop_brake_hold, "steer": 0.0})
                return

            self.stop_integral += v * self.ctrl_period
            brake_cmd = self.stop_brake_gain * self.stop_integral
            brake_cmd = max(self.stop_brake_hold, min(0.6, brake_cmd))

            self.get_logger().info(
                f"state=STOPPING, v={v:.2f}, d_goal={self._dist_to_goal():.2f}, brake={brake_cmd:.2f}",
                throttle_duration_sec=1.0
            )

            self._pub_cmd({"throttle": 0.0, "brake": brake_cmd, "steer": 0.0})
            return

        if self.tracker_state == self.STATE_STOPPED:
            self.get_logger().info(
                f"state=STOPPED, v={self.now_state['speed']:.2f}, d_goal={self._dist_to_goal():.2f}, "
                f"brake={self.stop_brake_hold:.2f}",
                throttle_duration_sec=1.0
            )
            self._pub_cmd({"throttle": 0.0, "brake": self.stop_brake_hold, "steer": 0.0})
            return


        ref_point = self._find_ref_point()
        steer_out = self.lat_ctrl.compute(self.now_state, ref_point, self.ctrl_period)
        accel_out = self.lon_ctrl.compute(self.now_state, ref_point, self.ctrl_period)

        ctrl_cmd = self._map2cmd(accel_out, steer_out)

        dist_to_goal = self._dist_to_goal()

        ########################debug###########################################
        v_ref = ref_point["speed"]
        speed_err = v_ref - self.now_state["speed"]
        # 真实加速度（由速度差分估计）
        if not hasattr(self, "prev_speed"):
            self.prev_speed = self.now_state["speed"]
        real_acc = (self.now_state["speed"] - self.prev_speed) / self.ctrl_period
        self.prev_speed = self.now_state["speed"]

        ########################debug###########################################
        self.get_logger().info(
            f"v={self.now_state['speed']:.2f},speed_err = {speed_err:.2f},acc = {accel_out:.2f},real_acc = {real_acc:.2f}  "
            f"d_goal={dist_to_goal:.2f}, th={ctrl_cmd['throttle']:.2f}, "
            f"br={ctrl_cmd['brake']:.2f}, st={ctrl_cmd['steer']:.2f}",
            throttle_duration_sec=1.0
        )
        self._pub_cmd(ctrl_cmd)

    def _on_odom(self, msg):

        # position
        self.now_state["x"] = msg.pose.pose.position.x
        self.now_state["y"] = msg.pose.pose.position.y

        # yaw     
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.now_state["yaw"] = math.atan2(siny_cosp, cosy_cosp)

        # speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.now_state["speed"] = math.sqrt(vx * vx + vy * vy + vz * vz)
    
        self.odom_ready = True

    def _load_ref_traj(self):
        pkg_share = get_package_share_directory("carla_vehicle_control")
        csv_path = os.path.join(pkg_share, "config", f"route_{self.route}.csv")

        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"route file not found: {csv_path}")
        
        self.ref_traj = []
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.ref_traj.append({
                    "s": float(row["s"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "yaw": float(row["yaw"]),
                    "kappa": float(row["kappa"]),
                    "speed": float(row["speed"]),
                })

    def _find_ref_point(self):
        search_window = 30
        start = self.idx_ref
        end = min(self.idx_ref + search_window, len(self.ref_traj))

        min_dist = float("inf")
        nearest_idx = self.idx_ref

        for i in range(start, end):
            dx = self.ref_traj[i]["x"] - self.now_state["x"]
            dy = self.ref_traj[i]["y"] - self.now_state["y"]
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        self.idx_ref = nearest_idx
        lookahead_pts = 8
        ref_idx = min(nearest_idx + lookahead_pts, len(self.ref_traj) - 1)

        return self.ref_traj[ref_idx]

    def _load_calibration_table(self):
        pkg_share = get_package_share_directory("carla_vehicle_control")
        csv_path = os.path.join(pkg_share, "config", "calibration_table.csv")

        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            rows = list(reader)

        self.calib_speed_bins = [float(x) for x in rows[0][1:]]
        self.calib_throttle_list = []
        self.calib_grid = []  # grid[i][j] = throttle_i 在 speed_bin_j 下的加速度，NaN->None
        for r in rows[1:]:
            self.calib_throttle_list.append(float(r[0]))
            vals = []
            for x in r[1:]:
                vals.append(None if x == "NaN" else float(x))
            self.calib_grid.append(vals)

        self.get_logger().info(
            f"calibration table loaded: {len(self.calib_throttle_list)} throttle levels, "
            f"{len(self.calib_speed_bins)} speed bins"
        )


    def _map2cmd(self, accel_out, steer_out):

        dead_zone = 0.05
        v_now = self.now_state["speed"]

        if accel_out >= 0.0:
            throttle_cmd = self._interp_2d(accel_out, v_now)
            throttle_cmd += 0.0153 * v_now
            brake_cmd = 0.0
        else:
            if abs(accel_out) < dead_zone:
                throttle_cmd = 0.0
                brake_cmd = 0.0
            else:
                throttle_cmd = 0.0
                brake_cmd = max(0.0, min(1.0, abs(accel_out) * self.BRAKE_K))

        steer_cmd = max(-1.0, min(1.0, steer_out))
        throttle_cmd = max(0.0, min(1.0, throttle_cmd))
        brake_cmd = max(0.0, min(1.0, brake_cmd))

        return {
            "throttle": throttle_cmd,
            "brake": brake_cmd,
            "steer": steer_cmd,
        }

    def _interp_2d(self, a_des, v_now):
        """二维查表：已知期望加速度和当前速度，插值得到油门开度。"""

        # step1: 每个throttle行，在v_now处插值得到对应加速度
        a_at_v = []
        for row in self.calib_grid:
            valid = [
                (self.calib_speed_bins[j], row[j])
                for j in range(len(self.calib_speed_bins))
                if row[j] is not None
            ]
            if len(valid) < 2:
                a_at_v.append(None)
                continue

            if v_now <= valid[0][0]:
                a_at_v.append(valid[0][1])
                continue
            if v_now >= valid[-1][0]:
                a_at_v.append(valid[-1][1])
                continue

            val = None
            for k in range(len(valid) - 1):
                x0, y0 = valid[k]
                x1, y1 = valid[k + 1]
                if x0 <= v_now <= x1:
                    val = y0 + (y1 - y0) * (v_now - x0) / (x1 - x0)
                    break
            a_at_v.append(val)

        # step2: 在 (a_at_v, throttle) 曲线上反查 a_des 对应的 throttle
        pairs = [
            (a_at_v[i], self.calib_throttle_list[i])
            for i in range(len(self.calib_throttle_list))
            if a_at_v[i] is not None
        ]
        if len(pairs) < 2:
            return 0.0

        pairs.sort(key=lambda x: x[0])

        if a_des <= pairs[0][0]:
            return pairs[0][1]
        if a_des >= pairs[-1][0]:
            return pairs[-1][1]

        for i in range(len(pairs) - 1):
            a0, t0 = pairs[i]
            a1, t1 = pairs[i + 1]
            if a0 <= a_des <= a1:
                return max(0.0, min(1.0, t0 + (t1 - t0) * (a_des - a0) / (a1 - a0)))

        return 0.0
    
    def _is_ready_to_stop(self):
        """
         检测是否准备停车
         return:
            要进入停车状态了: True
            不要进入停车状态了: False
        """
        if len(self.ref_traj) == 0:
            return False

        goal = self.ref_traj[-1]
        dx = goal["x"] - self.now_state["x"]
        dy = goal["y"] - self.now_state["y"]
        dist_to_goal = math.hypot(dx, dy)

        return dist_to_goal < self.stop_distance

    def _dist_to_goal(self):
        """
         检查停车距离
        """
        if len(self.ref_traj) == 0:
            return float("inf")
        goal = self.ref_traj[-1]
        dx = goal["x"] - self.now_state["x"]
        dy = goal["y"] - self.now_state["y"]
        return math.hypot(dx, dy)

    def _pub_cmd(self, ctrl_cmd):
        msg = CarlaEgoVehicleControl()

        msg.throttle = float(max(0.0, min(1.0, ctrl_cmd["throttle"])))
        msg.brake = float(max(0.0, min(1.0, ctrl_cmd["brake"])))
        msg.steer = float(max(-1.0, min(1.0, ctrl_cmd["steer"])))

        msg.hand_brake = False
        msg.reverse = False
        msg.manual_gear_shift = False
        msg.gear = 0

        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == "__main__":
    main()
