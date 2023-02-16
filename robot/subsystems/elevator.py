from commands2 import SubsystemBase
import rev

import constants

class Elevator(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        self.elevator_controller = rev.CANSparkMax(constants.k_elevator_motor_port, rev.CANSparkMax.MotorType.kBrushless)




