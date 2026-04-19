from math import sin, cos, atan2, sqrt, radians
from .base_controller import LateralController

class LatST(LateralController):

    def __init__(self, params: dict):

        # Stanley 增益参数
        self.k    = params.get("k", 0.5)       # 横向误差增益
        self.ks   = params.get("ks", 0.1)      # 速度软化系数，防低速除零

        # 转向限幅
        self.steer_limit   = params.get("steer_limit", 1.0)
        self.max_steer_rad = radians(params.get("max_steer_deg", 35.0))

        # 车辆参数
        self.wheelbase = params.get("wheelbase", 2.85)  # 轴距 (m)

    def compute(self, vehicle_state, ref_points, dt):
        
        x       = vehicle_state["x"]
        y       = vehicle_state["y"]
        yaw   = vehicle_state["yaw"]
        speed = vehicle_state.get("speed", 0.0)

        # 取最近点作为参考（Stanley 基于前轴最近点）
        ref_point = ref_points[0]
        x_ref   = ref_point["x"]
        y_ref   = ref_point["y"]
        yaw_ref = ref_point["yaw"]

        # 1. 前轴位置（车辆前方 wheelbase 处）
        fx = x + self.wheelbase * cos(yaw)
        fy = y + self.wheelbase * sin(yaw)

        # 2. 找前轴最近的路径点
        nearest = self._find_nearest(fx, fy, ref_points)
        x_near   = nearest["x"]
        y_near   = nearest["y"]
        yaw_near = nearest["yaw"]

        # 3. 航向误差 psi_e = 路径切线方向 - 车辆朝向
        psi_e = -(yaw_near - yaw)
        psi_e = atan2(sin(psi_e), cos(psi_e))  # 归一化[-pi, pi]

        # 4. 横向误差 e（前轴到最近路径点，带符号）
        #    投影到路径法方向：左正右负
        dx = fx - x_near
        dy = fy - y_near
        e = dx * (-sin(yaw_near)) + dy * cos(yaw_near)

        # 5. Stanley 公式
        #    delta = psi_e + arctan(k * e / (ks + v))
        steer_rad = psi_e + atan2(self.k * e, self.ks + abs(speed))

        steer = steer_rad / self.max_steer_rad
        steer = max(-self.steer_limit, min(self.steer_limit, steer))

        return steer
    

    def _find_nearest(self, fx, fy, ref_points):
        """找前轴最近的路径点"""
        min_dist = float("inf")
        nearest  = ref_points[0]
        for pt in ref_points:
            d = sqrt((pt["x"] - fx)**2 + (pt["y"] - fy)**2)
            if d < min_dist:
                min_dist = d
                nearest  = pt
        return nearest

    def reset(self):
        pass 