from commands2 import SubsystemBase
import rev

import constants

class Wrist(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.wrist_controller = rev.CANSparkMax(constants.k_wrist_motor_port, motor_type)




