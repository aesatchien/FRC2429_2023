import wpilib
import rev
from commands2 import SubsystemBase
from wpilib import MotorControllerGroup, PWMSparkMax, SmartDashboard
import constants

class SwerveDriveTrain(SubsystemBase):
    def __init__(self):
        super.__init__()
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.spark_neo_left_front_drive = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_left_front_turn = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_left_back_drive = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_left_back_turn = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_right_front_drive = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_right_front_turn = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_right_back_drive = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_right_back_turn = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)

        self.spark_neo_left_front_drive_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_left_front_drive)
        self.spark_neo_left_front_turn_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_left_front_turn)
        self.spark_neo_left_back_drive_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_left_back_drive)
        self.spark_neo_left_back_turn_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_left_back_turn)
        self.spark_neo_right_front_drive_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_right_front_drive)
        self.spark_neo_right_front_turn_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_right_front_turn)
        self.spark_neo_right_back_drive_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_right_back_drive)
        self.spark_neo_right_back_turn_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_right_back_turn)