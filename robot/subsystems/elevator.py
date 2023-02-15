from commands2 import SubsystemBase
import rev

import constants

class Elevator(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.elevator_controller = rev.CANSparkMax(constants.k_elevator_motor_port, motor_type)




