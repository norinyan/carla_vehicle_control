from math import pi, sin, cos, atan2
from .base_controller import LateralController

class LatPD(LateralController):

    def __init__(self, params: dict):

        self.kp = params.get("kp", 0.6)
        self.kd = params.get("kd", 0.08)
        self.k_yaw = params.get("k_yaw", 1.2)

        self.steer_limit = params.get("steer_limit", 1.0)

        # D项需要上次的误差
        self.prev_error = 0.0

    def compute(self, vehicle_state, ref_point, dt):

        # 当前状态
        x = vehicle_state["x"]
        y = vehicle_state["y"]
        yaw = vehicle_state["yaw"]

        # 参考状态
        x_ref = ref_point["x"]
        y_ref = ref_point["y"]
        yaw_ref = ref_point["yaw"]

        # 横向误差 ，发现越控制越偏离就e_d变-e_d
        dx = x - x_ref
        dy = y - y_ref
        e_d = -sin(yaw_ref)*dx + cos(yaw_ref)*dy

        # 航向误差
        e_yaw = -self._norm(yaw_ref - yaw)

        # 组合误差
        error = e_d + self.k_yaw * e_yaw

        # PD
        dt = max(dt, 1e-3)
        d_error = (error - self.prev_error) / dt
        self.prev_error = error

        # 曲率前馈
        kappa = ref_point.get("kappa", 0.0)
        steer_ff = - kappa * 0.5  # 系数先给0.5

        steer = self.kp * error + self.kd * d_error + steer_ff
        
        # 限幅
        steer = max(-self.steer_limit, min(self.steer_limit, steer))

        return steer

    def reset(self):
        self.prev_error = 0.0


    # 角度归一化
    def _norm(self, angle):

        angle = atan2(sin(angle), cos(angle))

        return angle
        
