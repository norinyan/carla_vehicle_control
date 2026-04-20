# -*- coding: utf-8 -*-

import cvxpy as cp
import numpy as np
from math import sin, cos, atan2, sqrt, radians
from .base_controller import LateralController

class LatMPC(LateralController):
    def __init__(self, params):
        self.N  = params.get("N",  15)     # 预测步长
        self.dt = params.get("dt",  0.1)

        self.L            = params.get("wheelbase",    2.85)
        self.max_steer_rad = radians(params.get("max_steer_deg", 35.0))

        # 代价权重
        q_ey          = params.get("q_ey",    1.0)
        q_eyaw        = params.get("q_eyaw",  2.0)
        self.Q        = np.diag([q_ey, q_eyaw])
        self.P        = np.diag([q_ey * 2.0, q_eyaw * 2.0])  # 终端代价适当加大
        self.R_delta  = params.get("R_delta",  0.1)
        self.R_ddelta = params.get("R_ddelta", 0.5)

        # 转角约束
        self.delta_min  = -self.max_steer_rad
        self.delta_max  =  self.max_steer_rad
        self.ddelta_min = -radians(params.get("ddelta_max_deg", 5.0))
        self.ddelta_max =  radians(params.get("ddelta_max_deg", 5.0))

        self.delta_prev = 0.0

        self._setup_mpc()


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

        # 2. 求解MPC输出
        delta_opt = self._solve_mpc(e_y, e_yaw)

        # 3. 更新历史
        self.delta_prev = delta_opt

        # 4. 输出steer
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

    def _solve_mpc(self, e_y, e_yaw):
        print(f"[MPC] e_y={e_y:.3f}, e_yaw={e_yaw:.3f}, delta_prev={self.delta_prev:.3f}")
        # 更新初始状态和模型矩阵
        A, B = self._build_model()
        self.x0_param.value         = np.array([e_y, e_yaw])
        self.A_param.value          = A
        self.B_param.value          = B
        self.delta_prev_param.value = self.delta_prev 

        # 求解
        self.mpc_problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)

        # 保底
        if self.u_var.value is None:
            print("[MPC] 求解失败，保持上一帧")
            return self.delta_prev

        delta_opt = float(self.u_var.value[0, 0])

        # 打印当前误差和求解结果
        print(f"[MPC] e_y={e_y:.3f}  e_yaw={e_yaw:.3f}  delta_prev={self.delta_prev:.3f}  delta_opt={delta_opt:.3f}")
        
        # 打印预测时域控制序列（前5步）
        u_seq  = self.u_var.value[0, :5].round(3)
        print(f"[MPC] u_seq  (前5步): {u_seq}")

        # 打印预测时域状态序列（前5步）
        ey_seq   = self.x_var[0, :6].value.round(3)
        eyaw_seq = self.x_var[1, :6].value.round(3)
        print(f"[MPC] e_y预测 (前5步): {ey_seq}")
        print(f"[MPC] eyaw预测(前5步): {eyaw_seq}")

        # 取第一步最优前轮转角
        return delta_opt

    def _build_model(self):
        v = max(self.vehicle_state["speed"], 0.5)
        A = np.array([[1.0, v * self.dt],
                    [0.0, 1.0       ]])
        B = np.array([[0.0                  ],
                    [- v * self.dt / self.L ]])
        return A, B

    def _setup_mpc(self):

        # 1: 声明优化变量   
        self.x_var = cp.Variable((2, self.N + 1))   # 状态序列 [e_y, e_yaw]
        self.u_var = cp.Variable((1, self.N))        # 控制序列 [delta]
        
        # 2: 声明每帧更新的参数
        self.x0_param         = cp.Parameter(2)        # 初始误差状态
        self.A_param          = cp.Parameter((2, 2))   # 状态矩阵
        self.B_param          = cp.Parameter((2, 1))   # 控制矩阵
        self.delta_prev_param = cp.Parameter()         # 上一帧转角

        # 3: 构建代价函数和约束

        cost        = 0
        constraints = [self.x_var[:, 0] == self.x0_param]  # 初始状态约束

        for k in range(self.N):
            # 转角增量（k=0时参考上一帧，k>0时参考上一步）
            delta_ref = self.delta_prev_param if k == 0 else self.u_var[0, k - 1]
            ddelta    = self.u_var[0, k] - delta_ref

            # 代价：状态偏差 + 转角幅值 + 转角增量
            cost += cp.quad_form(self.x_var[:, k], self.Q)
            cost += self.R_delta  * cp.square(self.u_var[0, k])
            cost += self.R_ddelta * cp.square(ddelta)

            # 约束：动力学 + 转角幅值 + 转角增量
            constraints += [self.x_var[:, k + 1] == self.A_param @ self.x_var[:, k] + self.B_param @ self.u_var[:, k]]
            constraints += [self.u_var[0, k] >= self.delta_min, self.u_var[0, k] <= self.delta_max]
            constraints += [ddelta >= self.ddelta_min,          ddelta <= self.ddelta_max]

        # 终端代价
        cost += cp.quad_form(self.x_var[:, self.N], self.P)

        # 4: 构建问题
        self.mpc_problem = cp.Problem(cp.Minimize(cost), constraints)



