from dataclasses import dataclass


@dataclass
class CarSignals:
    elapsed_time: float  # seconds
    acc_x: float         # m/s^2
    acc_y: float         # m/s^2
    yaw_rate: float      # radians per second
    steer_angle: float   # degrees
    wheel_speed_rr: float  # rpm
    wheel_speed_rl: float  # rpm
    wheel_speed_fr: float  # rpm
    wheel_speed_fl: float  # rpm

@dataclass
class State:
    x: float       # meters
    y: float       # meters
    yaw: float     # radians
    vx: float      # m/s