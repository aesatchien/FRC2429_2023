import wpilib
import rev
from commands2 import SubsystemBase
from wpilib import MotorControllerGroup, PWMSparkMax, SmartDashboard
import constants
import math

class SwerveDrivetrain(SubsystemBase):
    def __init__(self, container, L, W) -> None:
        '''Initializes a swerve drivetrain analogous to drivetrain.py
        L: The distance between the front and back wheels.
        W: The distance between the left and right wheels.
        '''
        super.__init__()

        self.container = container
        self.L = L
        self.W = W
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

        # Put conversion factors here

        self.spark_PID_controller_left_front_drive = rev.CANSparkMax.getPIDController(self.spark_neo_left_front_drive)
        self.spark_PID_controller_left_front_turn = rev.CANSparkMax.getPIDController(self.spark_neo_left_front_turn)
        self.spark_PID_controller_left_back_drive = rev.CANSparkMax.getPIDController(self.spark_neo_left_back_drive)
        self.spark_PID_controller_left_back_turn = rev.CANSparkMax.getPIDController(self.spark_neo_left_back_turn)
        self.spark_PID_controller_right_front_drive = rev.CANSparkMax.getPIDController(self.spark_neo_right_front_drive)
        self.spark_PID_controller_right_front_turn = rev.CANSparkMax.getPIDController(self.spark_neo_right_front_turn)
        self.spark_PID_controller_right_back_drive = rev.CANSparkMax.getPIDController(self.spark_neo_right_back_drive)
        self.spark_PID_controller_right_back_turn = rev.CANSparkMax.getPIDController(self.spark_neo_right_back_turn)


    def arcade_drive(self, fwd, strafe, rot):
        r = math.sqrt((self.L * self.L) + (self.W * self.W))
        fwd *= -1
        a = strafe - rot * (self.L / r)
        b = strafe + rot * (self.L / r)
        c = fwd - rot * (self.W / r)
        d = fwd + rot * (self.W / r)

        backRightSpeed = math.sqrt((a * a) + (d * d))
        backLeftSpeed = math.sqrt((a * a) + (c * c))
        frontRightSpeed = math.sqrt((b * b) + (d * d))
        frontLeftSpeed = math.sqrt((b * b) + (c * c))

        backRightAngle = math.atan2(a, d) / math.pi
        backLeftAngle = math.atan2(a, c) / math.pi
        frontRightAngle = math.atan2(b, d) / math.pi
        frontLeftAngle = math.atan2(b, c) / math.pi

        self.spark_PID_controller_left_front_drive.setReference(frontLeftSpeed, rev.CANSparkMax.ControlType.kPosition)
        self.spark_PID_controller_left_front_turn.setReference(frontLeftAngle, rev.CANSparkMax.ControlType.kPosition)
        self.spark_PID_controller_left_back_drive.setReference(backLeftSpeed, rev.CANSparkMax.ControlType.kPosition)
        self.spark_PID_controller_left_back_turn.setReference(backLeftAngle, rev.CANSparkMax.ControlType.kPosition)
        self.spark_PID_controller_right_front_drive.setReference(frontRightSpeed, rev.CANSparkMax.ControlType.kPosition)
        self.spark_PID_controller_right_front_turn.setReference(frontRightAngle, rev.CANSparkMax.ControlType.kPosition)
        self.spark_PID_controller_right_back_drive.setReference(backRightSpeed, rev.CANSparkMax.ControlType.kPosition)
        self.spark_PID_controller_right_back_turn.setReference(backRightAngle, rev.CANSparkMax.ControlType.kPosition)