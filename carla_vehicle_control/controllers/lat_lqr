# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, atan2, sqrt, radians
from .base_controller import LateralController

class LatLQR(LateralController):
    def __init__(self, params):
        self.L             = params.get("wheelbase",    2.85)
        self.max_steer_rad = radians(params.get("max_steer_deg", 35.0))

        # Riccati 迭代参数
        self.max_iter = params.get("max_iter", 100)
        self.eps      = params.get("eps",      1e-4)

        # 状态代价矩阵 Q：惩罚 e_y 和 e_yaw
        q_ey   = params.get("q_ey",   1.0)
        q_eyaw = params.get("q_eyaw", 1.0)
        self.Q = np.diag([q_ey, q_eyaw])

        # 控制代价矩阵 R：惩罚转角幅值
        r_delta  = params.get("r_delta", 1.0)
        self.R   = np.array([[r_delta]])



    def compute(self, vehicle_state, ref_points, dt):
        """
        vehicle_state : {x, y, yaw, speed}
        ref_points    : 从最近点开始往前的 N 个参考点，每点含 {x, y, yaw, kappa, speed, s}
        dt            : 控制周期 (s)
        return        : steer，范围 [-1.0, 1.0]
        """
        self.dt             = dt
        self.vehicle_state  = vehicle_state
        self.ref_points     = ref_points

        # 1. 计算误差状态
        e_y, e_yaw = self._compute_error()

        # 2. 构建模型
        A, B = self._build_model()

        # 3. 求解LQR增益
        K = self._solve_lqr(A, B)

        # 4. 计算最优前轮转角
        x = np.array([e_y, e_yaw])
        delta_opt = -K @ x
        delta_opt = float(delta_opt)

        # 5. 输出steer
        steer = delta_opt / self.max_steer_rad
        steer = max(-1.0, min(1.0, steer))
        return steer
        
    def _compute_error(self):
        """
        计算当前横向偏差 e_y 和航向偏差 e_yaw。
        使用最近参考点（ref_points[0]）作为误差计算基准。

        e_y   : 车辆位置投影到参考点法向的距离（左正右负）
        e_yaw : 车辆航向 - 参考航向，归一化到 [-pi, pi]
        """
        x,   y   = self.vehicle_state["x"],   self.vehicle_state["y"]
        yaw       = self.vehicle_state["yaw"]
        ref        = self.ref_points[0]
        x_ref,  y_ref   = ref["x"], ref["y"]
        yaw_ref       = ref["yaw"]

        dx  = x - x_ref
        dy  = y - y_ref
        e_y = -dx * sin(yaw_ref) + dy * cos(yaw_ref)

        e_yaw = yaw - yaw_ref
        e_yaw = atan2(sin(e_yaw), cos(e_yaw))

        return e_y, e_yaw

    def _solve_lqr(self, A, B):
        # 初始化 P 为状态代价矩阵 Q
        P = self.Q.copy()

        for _ in range(self.max_iter):
            # Riccati 迭代核心公式
            # P_new = Q + A'PA - A'PB · (R + B'PB)^{-1} · B'PA
            P_new = (self.Q
                    + A.T @ P @ A
                    - A.T @ P @ B @ np.linalg.pinv(self.R + B.T @ P @ B) @ B.T @ P @ A)

            # 收敛判断：P 的最大元素变化量小于阈值则退出
            if abs(P_new - P).max() < self.eps:
                break
            P = P_new

        # 由收敛后的 P 计算最优反馈增益 K
        # K = (R + B'PB)^{-1} · B'PA
        K = np.linalg.pinv(self.R + B.T @ P @ B) @ B.T @ P @ A
        return K  # shape: (1, 2)，作用于 [e_y, e_yaw]

    def _build_model(self):
        v = max(self.vehicle_state["speed"], 0.5)
        A = np.array([[1.0, v * self.dt],
                    [0.0, 1.0       ]])
        B = np.array([[0.0                  ],
                    [- v * self.dt / self.L ]])
        return A, B
