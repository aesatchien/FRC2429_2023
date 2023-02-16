from commands2 import SubsystemBase
import rev

import constants

class Arm(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        self.arm_controller = rev.CANSparkMax(constants.k_arm_motor_port, rev.CANSparkMax.MotorType.kBrushless)




