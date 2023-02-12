from commands2 import SubsystemBase
import rev

import constants

class TurretSubsystem(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.counter = 0

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        spark_neo = rev.CANSparkMax(constants.k_turret_motor_port, motor_type)
        encoder = spark_neo.getAlternateEncoder(1)
        controller = spark_neo.getPIDController()

    def set_voltage(self, voltage):
        pass
        # self.controller.setReference()