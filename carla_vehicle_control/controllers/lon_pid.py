import math
import yaml
from .base_controller import LongitudinalController


class LonPID(LongitudinalController):
    def __init__(self, params: dict):

        self.kp = params.get("kp", 0.55)
        self.ki = params.get("ki", 0.01)
        self.kd = params.get("kd", 0.05)

        self.a_limit = params.get("a_limit", 3.0)  # 最大加速度
        self.i_limit = params.get("i_limit", 1.0)  # 积分限幅
        
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, vehicle_state, ref_point, dt):
        
        v = vehicle_state["speed"]
        v_ref = ref_point["speed"]
        
        dt = max(dt, 1e-3)
        
        # 速度误差
        speed_err = v_ref - v

        # 积分项（限幅防止饱和）
        self.integral += speed_err * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        
        # 微分项
        d_error = (speed_err - self.prev_error) / dt
        self.prev_error = speed_err
        
        # PID
        a_cmd = self.kp * speed_err + self.ki * self.integral + self.kd * d_error
        
        # 限幅
        a_cmd = max(-self.a_limit, min(self.a_limit, a_cmd))
        
        return a_cmd

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0




