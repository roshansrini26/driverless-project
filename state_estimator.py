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
    STANDSTILL_RPM = 5.0
    YAW_SOURCE = "gyro"

    def __init__(self):
        self.prev_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        #gyro bias estimation
        self.yaw_rate_count = 0
        self.yaw_rate_sum = 0.0
        self.yaw_rate_bias = 0.0

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
        if (abs(meas.wheel_speed_rl) < self.STANDSTILL_RPM 
            and abs(meas.wheel_speed_rr) < self.STANDSTILL_RPM 
            and abs(meas.wheel_speed_fl) < self.STANDSTILL_RPM 
            and abs(meas.wheel_speed_fr) < self.STANDSTILL_RPM):
                self.yaw_rate_sum += meas.yaw_rate
                self.yaw_rate_count += 1
                self.yaw_rate_bias = self.yaw_rate_sum / self.yaw_rate_count

        if self.YAW_SOURCE == "steer":
            yaw_rate = vx * math.tan(math.radians(meas.steer_angle)) / self.WHEELBASE
        else:
            yaw_rate = meas.yaw_rate - self.yaw_rate_bias
              
        self.yaw += yaw_rate * dt
        #self.yaw += (meas.yaw_rate - self.yaw_rate_bias) * dt

        #position
        self.x = self.x + vx * math.cos(self.yaw) * dt
        self.y = self.y + vx * math.sin(self.yaw) * dt

        return State(x=self.x, y=self.y, yaw=self.yaw, vx=vx)