from datatypes import CarSignals, State

import math
class StateEstimator:
    """
    Baseline state estimator template for applicants.

    Contract (inputs/outputs)

    - predict(meas: CarSignals) -> State
      - meas: one timestep of sensor data (see `types.py`)
      - returns: State containing:
          x [m]
          y [m]
          yaw [rad]
          vx [m/s]
    """

    TIRE_RADIUS = 0.2032  #fixed tyre radius constant
    TRACK = 1.23  # m
    WHEELBASE = 1.53  # m
    CAR_MASS = 192  # kg

    def __init__(self):
        self.prev_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def predict(self, meas: CarSignals) -> State:
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = meas.elapsed_time - self.prev_time
        self.prev_time = meas.elapsed_time

        #speed
        v_rl = meas.wheel_speed_rl / 60 * 2 * math.pi * self.TIRE_RADIUS
        v_rr = meas.wheel_speed_rr / 60 * 2 * math.pi * self.TIRE_RADIUS
        vx = (v_rl + v_rr) / 2

        #heading
        self.yaw = self.yaw + meas.yaw_rate * dt

        #position
        self.x = self.x + vx * math.cos(self.yaw) * dt
        self.y = self.y + vx * math.sin(self.yaw) * dt

        return State(x=self.x, y=self.y, yaw=self.yaw, vx=vx)