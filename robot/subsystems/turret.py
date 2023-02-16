"""
Turret subsystem
Shares the following with networktables:  turret_angle
"""

from commands2 import SubsystemBase
import rev

import constants
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Turret(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        self.turret_controller = rev.CANSparkMax(constants.k_turret_motor_port, rev.CANSparkMax.MotorType.kBrushless)




