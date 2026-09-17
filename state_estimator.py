from types import CarSignals, State

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

    TIRE_RADIUS = 0.20032  # m
    TRACK = 1.23  # m
    WHEELBASE = 1.53  # m
    CAR_MASS = 192  # kg

    def __init__(self):
        pass

    def predict(self, meas: CarSignals) -> State:
        pass