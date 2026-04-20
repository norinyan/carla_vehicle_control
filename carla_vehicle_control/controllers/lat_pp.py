from math import pi, sin, cos, atan2, sqrt, tan, radians
from .base_controller import LateralController

class LatPP(LateralController):

    def __init__(self, params:dict):

        # 前视距离参数
        self.k_ld     = params.get("k_ld", 0.15)      # 前视距离速度增益
        self.ld_min   = params.get("ld_min", 2.0)      # 最小前视距离 (m)
        self.ld_max   = params.get("ld_max", 10.0)     # 最大前视距离 (m)

        # 车辆参数
        self.wheelbase = params.get("wheelbase", 2.85) # 轴距 (m)，Lincoln MKZ 约 2.85m
       
        # 转向限幅（与 lat_pd 保持一致，单位同为归一化 [-1, 1]）
        self.steer_limit = params.get("steer_limit", 1.0)

        # 最大物理转向角（用于归一化），Lincoln MKZ 约 ±70° 方向盘 → 前轮约 ±35°
        self.max_steer_rad = radians(params.get("max_steer_deg", 35.0))

    def compute(self, vehicle_state, ref_points, dt):

        x       = vehicle_state["x"]
        y       = vehicle_state["y"]
        yaw     = vehicle_state["yaw"]
        speed   = vehicle_state.get("speed", 0.0)

        # 1. 计算自适应前视距离
        ld = self._compute_lookahead(speed)

        # 2. 找前视点(从最近点向前搜索)
        # 
        tx, ty = self._find_target(x, y, ld, ref_points)

        # 3. 计算alpha
        dy = ty - y
        dx = tx - x
        alpha = -(atan2(dy, dx) - yaw) 
        alpha = atan2(sin(alpha), cos(alpha))   # 归一化[-pi, pi]


        # 4.计算u = delta = arctan(2L·sin(alpha) / ld) 
        actual_ld = max(sqrt(dx**2 + dy**2), 1e-3)
        steer_rad = atan2(2.0 * self.wheelbase * sin(alpha), actual_ld)
        # 临时debug
        print(f"ld={ld:.2f}, tx={tx:.2f}, ty={ty:.2f}, alpha={alpha:.3f}, steer_rad={steer_rad:.3f}")

        
        steer = steer_rad / self.max_steer_rad
        steer = max(-self.steer_limit, min(self.steer_limit, steer))

        return steer
    
    def _compute_lookahead(self, speed):

        ld = self.ld_min + self.k_ld * abs(speed)
        return min(self.ld_max, ld)

    def _find_target(self, x, y, ld, ref_points):

        for pt in ref_points:
            d = sqrt((pt["x"] - x)**2 + (pt["y"] - y)**2)
            if d >= ld:
                return pt["x"], pt["y"]
            
    
        return ref_points[-1]["x"], ref_points[-1]["y"]
    
    def reset(self):
        pass  
