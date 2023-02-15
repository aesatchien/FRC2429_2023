from commands2 import SubsystemBase
import rev

import constants

class Turret(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.turret_controller = rev.CANSparkMax(constants.k_turret_motor_port, motor_type)




