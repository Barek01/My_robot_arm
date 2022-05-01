#! /usr/bin/env python3

import typing as tp
from math import pi, isclose, degrees
from dataclasses import dataclass
from numpy import clip
import can

from tinymovr import Tinymovr
from tinymovr.iface.can_bus import CANBus, guess_channel
from tinymovr.units import get_registry

import rospy

ureg = get_registry()
Amps = ureg.ampere
s = ureg.second
tick = ureg.tick


TOLERANCE = 1e-6
@dataclass
class MotorConfig:
    """
    @class MotorConfig: motor board specific parameters
    """
    kv: float
    max_current: float
    position_gain: float
    velocity_gain: float
    can_id: int
    direction: int = 1.0
    name: str = 'unknown'


def get_bus() -> CANBus:
    channel = guess_channel(bustype_hint='slcan')
    can_bus: can.Bus = can.Bus(bustype='slcan',
                                   channel=channel,
                                   bitrate=1000000)
    iface = CANBus(can_bus)
    return iface

class Motor:
    def __init__(self, config: MotorConfig, bus : CANBus):
        self.can_id = config.can_id
        self.kt = 30.0 / (pi * config.kv)
        self.max_current = config.max_current
        self.position_gain = config.position_gain
        self.velocity_gain = config.velocity_gain
        self.direction = config.direction
        self.integrator_gain = 0.0
        self.name = config.name
        self.init_driver(bus)
        self.update_config()
        self.is_running = False

    def init_driver(self, can_bus: CANBus) -> None:
        self.tm = Tinymovr(node_id=self.can_id, iface=can_bus)

        assert self.tm.motor_config.flags == 1 , "le moteur n'est pas configuré se referer au readme hal"
        resolution = 2**13
        self.scale = (2 * pi) / resolution * self.direction
        #self.integrator_gain = self.tm.integrator_gains.magnitude

    def start(self):
        self.tm.position_control()
        self.is_running = True

    def stop(self):
        if self.is_running:
            self.tm.set_state(state=0)
            print(f"{self.name} stopped")
            self.is_running = False

    def update_config(self) -> None:
        """
        Update board parameter of motor board based on config.
        """
        changed = self.update_max_current()
        changed = changed or self.update_gains()
        if changed:
            self.tm.save_config()

    def update_max_current(self) -> bool:
        changed = False
        limits = self.tm.limits
        max_current = limits.current
        max_vel = limits.velocity
        if not isclose(self.max_current, max_current.magnitude, rel_tol=TOLERANCE):
            self.tm.set_limits(velocity=max_vel, current=self.max_current)
            print(f"{self.name}: Changed current limit to {self.max_current}, was {max_current.magnitude}")
            changed = True
        return changed

    def update_gains(self) -> bool:
        gains_changed = False
        gains = self.tm.gains
        if not isclose(self.position_gain, gains.position.magnitude, rel_tol=TOLERANCE):
            print(f"{self.name}: changed position gain to {self.position_gain}, was {gains.position.magnitude}")
            gains_changed = True
        if not isclose(self.velocity_gain, gains.velocity.magnitude, rel_tol=TOLERANCE):
            print(f"{self.name}: changed velocity gain to {self.velocity_gain}, was {gains.velocity.magnitude}")
            gains_changed = True

        if gains_changed:
            self.tm.set_gains(position=self.position_gain, velocity=self.velocity_gain)

        return gains_changed

    def position(self) -> float:
        ticks = self.tm.encoder_estimates.position
        return (ticks * self.scale).magnitude

    def state(self) -> tp.Tuple[float, float]:
        state = self.tm.encoder_estimates
        position = state.position * self.scale
        velocity = state.velocity * self.scale
        return position.magnitude, velocity.magnitude

    def set_torque_target(self, torque: float) -> None:
        iq = torque / self.kt
        iq = clip(iq, -self.max_current, self.max_current)
        self.tm.set_cur_setpoint(iq * Amps)

    def set_kinematic_target(self, position : float, velocity : float = 0.0) -> None:
        target_pos = int(position  / self.scale)
        # Tinymvr uses velocity targets in decaticks/s
        target_velocity = int(velocity / self.scale) // 10
        self.tm.set_pos_setpoint(target_pos, target_velocity)

    def set_position_target(self, position : float) -> None:
        target = int(position  / self.scale)
        self.tm.set_pos_setpoint(target)

    def set_gains(self, position: float, velocity : float, integrator: float = 0) -> None:
        self.position_gain = position
        self.velocity_gain = velocity
        #self.integrator_gain = integrator
        self.tm.set_gains(position, velocity)
        #self.tm.set_integrator_gains(integrator)
        rospy.loginfo(f"Motor params updated: pos {position}, vel {velocity}, vel integrator: {integrator}")
