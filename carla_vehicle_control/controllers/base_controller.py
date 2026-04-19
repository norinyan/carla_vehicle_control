from abc import ABC, abstractmethod

class LateralController(ABC):
    @abstractmethod
    def compute(self, vehicle_state: dict, ref_point: list, dt: float) -> float:
        """
        vehicle_state: {x, y, yaw, speed}
        ref_point:     list of {x, y, yaw, kappa, speed, s}，从当前最近点开始往前的N个点
        dt:            控制周期(秒)
        return: steer，范围[-1.0, 1.0]
        """
        pass

class LongitudinalController(ABC):
    @abstractmethod
    def compute(self, vehicle_state: dict, ref_point: list, dt: float) -> float:
        """
        vehicle_state: {x, y, yaw, speed}
        ref_point:     list of {x, y, yaw, kappa, speed, s}，从当前最近点开始往前的N个点
        dt:            控制周期(秒)
        return: a_cmd (m/s^2)
        """
        pass