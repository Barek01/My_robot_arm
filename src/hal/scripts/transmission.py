#! /usr/bin/env python3

import typing as tp
from dataclasses import dataclass

@dataclass
class TransmissionConfig:
    ratio : float = 1.0
    zero : float = 0.0
    low : float = 0.0
    high : float = 0.0
@dataclass
class Kinematic:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0

class Transmission:
    """
    @class Transmission: A simple mechanical transmission
           which is a actually only a reduction.
    """
    def __init__(self, config : TransmissionConfig):
        self.ratio = config.ratio
        self.zero = config.zero
        self.low = config.low
        self.high = config.high

    def clamp_joint(self, state : Kinematic) -> Kinematic:
        if state.position <= self.low:
            state.position = self.low
        elif state.position >= self.high:
            state.position = self.high
        return state

    def joint_to_actuator(self, input: Kinematic) -> Kinematic:
        return Kinematic(input.position * self.ratio + self.zero,
                         input.velocity * self.ratio,
                         input.torque / self.ratio)

    def actuator_to_joint(self, input: Kinematic) ->  Kinematic:
        joint = Kinematic((input.position - self.zero) / self.ratio,
                         input.velocity / self.ratio,
                         input.torque * self.ratio)
        return joint

    def compute_zero(self, low : tp.Optional[float] = None, high : tp.Optional[float] = None) -> None:
        # A transmission in a function: Y = F(x)
        # Ymax = F(xmax)
        # Y0 = F(x0)
        #    # x0 = F^-1(Y0)
        #     Ymax -Y0 = F(xmax) - F(x0)

        #F^1 (ymax) = xmax - x0
        # x0 = xmax - F^1 (ymax)
        self.zero = high - self.high * self.ratio
