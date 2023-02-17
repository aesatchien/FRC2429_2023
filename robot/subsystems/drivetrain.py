from commands2 import SubsystemBase
from wpilib.drive import DifferentialDrive
from wpilib import MotorControllerGroup, PWMSparkMax
import rev
import navx

#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim

import constants

class Drivetrain(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize sensors - use the navx for headings
        self.navx = navx.AHRS.create_spi()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.spark_neo_left_front = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_left_back = rev.CANSparkMax(constants.k_left_motor2_port, motor_type)
        self.spark_neo_right_front = rev.CANSparkMax(constants.k_right_motor1_port, motor_type)
        self.spark_neo_right_back = rev.CANSparkMax(constants.k_right_motor2_port, motor_type)

        self.left_motors = MotorControllerGroup(self.spark_neo_left_front, self.spark_neo_left_back)
        self.right_motors = MotorControllerGroup(self.spark_neo_right_front, self.spark_neo_right_back)
        self.right_motors.setInverted(True)

        self.drive = DifferentialDrive(self.left_motors, self.right_motors)

        # add two dummy PWMs so we can track the SparkMax in the sim (should be updated in sim periodic)
        self.dummy_motor_left = PWMSparkMax(1)
        self.dummy_motor_right = PWMSparkMax(3)

    def arcade_drive(self, fwd, rot):
        self.drive.arcadeDrive(fwd, rot, squareInputs=True)
        # need to update the simulated PWMs here
        self.dummy_motor_left.set(self.spark_neo_left_front.get())
        self.dummy_motor_right.set(self.spark_neo_right_front.get())



