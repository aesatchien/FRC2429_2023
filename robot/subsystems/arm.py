"""
Arm subsystem
Shares the following with networktables:  arm_extension
"""

from commands2 import SubsystemBase
import rev

import constants
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim

class Arm(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        self.arm_controller = rev.CANSparkMax(constants.k_arm_motor_port, rev.CANSparkMax.MotorType.kBrushless)




