# -*- coding: utf-8 -*-
 
import cvxpy as cp
import numpy as np
from math import cos, sin
from .base_controller import LongitudinalController

class LonMPC(LongitudinalController):
    def __init__(self, params):


        # 参数
        self.N  = params.get("N",  15)       # 预测步长
        self.dt = params.get("dt", 0.1)      # 控制周期

        q_es    = params.get("q_es", 1.0)
        q_ev    = params.get("q_ev", 2.0)
        self.Q      = np.diag([q_es, q_ev])
        self.P      = np.diag([q_es * 2.0, q_ev * 2.0])
        self.R_a    = params.get("R_a",    0.1)
        self.R_da   = params.get("R_da",   0.5)
        self.a_min  = params.get("a_min", -3.0)
        self.a_max  = params.get("a_max",  1.5)
        self.da_max = params.get("da_max", 1.0)
        self.da_min = -self.da_max

        self.a_prev = 0.0
        
        self._setup_mpc()

    def compute(self, vehicle_state, ref_points, dt):

        
        """
        vehicle_state : {x, y, yaw, speed}
        ref_points    : 参考点列表，每点含 {x, y, yaw, kappa, speed, s}
        dt            : 控制周期 (s)
        return        : a_cmd (m/s²)
        """

        self.dt            = dt
        self.vehicle_state = vehicle_state
        self.ref_points    = ref_points

        # 1. 计算误差
        e_s, e_v = self._compute_error()

        # 2. 求解mpc
        a_opt = self._solve_mpc(e_s, e_v)

        # 3. 更新历史
        self.a_prev = a_opt

        return a_opt
        

    def _compute_error(self):
        
        x   = self.vehicle_state["x"]
        y   = self.vehicle_state["y"]
        v   = self.vehicle_state["speed"]

        ref     = self.ref_points[0]
        x_ref   = ref["x"]
        y_ref   = ref["y"]
        yaw_ref = ref["yaw"]
        v_ref   = ref["speed"]

        dx  = x_ref - x
        dy  = y_ref - y
        e_s = dx * cos(yaw_ref) + dy * sin(yaw_ref)
 
        e_v = v_ref - v
 
        return e_s, e_v

    
    def _build_model(self):
        """
        构建离散线性模型：
        状态 x = [e_s, e_v]，控制 u = [a]
 
        e_s(k+1) = e_s(k) + e_v(k) * dt
        e_v(k+1) = e_v(k) + a(k)   * dt
 
        A = [[1, dt],   B = [[0 ],
             [0,  1]]        [dt]]
        """
        dt = self.dt
        A = np.array([[1.0, dt ],
                      [0.0, 1.0]])
        B = np.array([[0.0],
                      [-dt ]])
        return A, B

    def _setup_mpc(self):
        
        # 1. 优化变量
        self.x_var = cp.Variable((2, self.N + 1))  # 状态序列 [e_s, e_v]
        self.u_var = cp.Variable((1, self.N))       # 控制序列 [a]

        # 2. 每帧更新的参数
        self.x0_param      = cp.Parameter(2)        # 初始误差状态 [e_s, e_v]
        self.A_param       = cp.Parameter((2, 2))   # 状态矩阵
        self.B_param       = cp.Parameter((2, 1))   # 控制矩阵
        self.a_prev_param  = cp.Parameter()         # 上一帧加速度

        # 3. 代价函数与约束
        cost        = 0
        constraints = [self.x_var[:, 0] == self.x0_param] 

        for k in range(self.N):
            # jerk：k=0 时参考上一帧，k>0 时参考上一步
            a_ref = self.a_prev_param if k == 0 else self.u_var[0, k - 1]
            da    = self.u_var[0, k] - a_ref
 
            # 代价：状态误差 + 加速度幅值 + jerk
            cost += cp.quad_form(self.x_var[:, k], self.Q)
            cost += self.R_a  * cp.square(self.u_var[0, k])
            cost += self.R_da * cp.square(da)
 
            # 约束：动力学 + 加速度幅值 + jerk
            constraints += [self.x_var[:, k + 1] == self.A_param @ self.x_var[:, k] + self.B_param @ self.u_var[:, k]]
            constraints += [self.u_var[0, k] >= self.a_min, self.u_var[0, k] <= self.a_max]
            constraints += [da >= self.da_min, da <= self.da_max]
 
        # 终端代价
        cost += cp.quad_form(self.x_var[:, self.N], self.P)
 
        # 4. 构建问题
        self.mpc_problem = cp.Problem(cp.Minimize(cost), constraints)

    def _solve_mpc(self, e_s, e_v):
        """
        填入当前误差状态，求解 MPC，返回第一步最优加速度。
        """
        print(f"[LonMPC] e_s={e_s:.3f}, e_v={e_v:.3f}, a_prev={self.a_prev:.3f}")
 
        # 更新参数
        A, B = self._build_model()
        self.x0_param.value     = np.array([e_s, e_v])
        self.A_param.value      = A
        self.B_param.value      = B
        self.a_prev_param.value = self.a_prev
 
        # 求解
        self.mpc_problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
 
        # 求解失败保底
        if self.u_var.value is None:
            print("[LonMPC] 求解失败，保持上一帧")
            return self.a_prev
 
        a_opt = float(self.u_var.value[0, 0])
 
        # 打印预测序列（前5步）
        u_seq   = self.u_var.value[0,   :5].round(3)
        es_seq  = self.x_var[0, :6].value.round(3)
        ev_seq  = self.x_var[1, :6].value.round(3)
        print(f"[LonMPC] a_opt={a_opt:.3f}")
        print(f"[LonMPC] u_seq  (前5步): {u_seq}")
        print(f"[LonMPC] e_s预测(前5步): {es_seq}")
        print(f"[LonMPC] e_v预测(前5步): {ev_seq}")
 
        return a_opt
 
    def reset(self):
        self.a_prev = 0.0


