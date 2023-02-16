from commands2 import SubsystemBase
import rev

import constants

class Turret(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        self.turret_controller = rev.CANSparkMax(constants.k_turret_motor_port, rev.CANSparkMax.MotorType.kBrushless)




