"""
Wrist subsystem
Shares the following with networktables:  wrist_position
"""
from commands2 import SubsystemBase
import rev
import wpilib
import constants
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Wrist(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.wrist_controller = rev.CANSparkMax(constants.k_wrist_motor_port, motor_type)

        # wrist should probably have four positions that we need to map out
        self.positions = {'stow':0, 'score':0, 'flat':0, 'floor':0}



