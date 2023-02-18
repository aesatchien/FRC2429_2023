"""
Arm subsystem
Shares the following with networktables:  arm_extension

Arm needs to
1) pull back all the way for stow
2) have a controllable distance from stow to fully extended
3) can probably trust the default encoder for the distance -

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




