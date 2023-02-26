"""
Wrist subsystem
Shares the following with networktables:  wrist_position
"""
from commands2 import SubsystemBase
import rev
import wpilib
from wpilib import SmartDashboard
import constants
from misc.configure_controllers import configure_sparkmax
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Wrist(SubsystemBase):
    # wrist should probably have four positions that we need to map out
    positions = {'stow': 125, 'score': 45, 'flat': 0, 'floor': -30}

    def __init__(self):
        super().__init__()
        self. counter = 0
        # defining angles so 0 is horizontal
        self.max_angle = 125  # call all the way up 125 degrees  todo: remeasure
        self.min_angle = -30

        # initialize motors
        self.wrist_controller = rev.CANSparkMax(constants.k_wrist_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.wrist_controller.setInverted(True)  # verified that this is true for the directions we want
        self.sparkmax_encoder = self.wrist_controller.getEncoder()

        # update sparkmax with appropriate system gains and constraints
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_wrist_encoder_conversion_factor)  # mm per revolution
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_wrist_encoder_conversion_factor)  # necessary for smartmotion to behave
        self.pid_controller = self.wrist_controller.getPIDController()
        configure_sparkmax(sparkmax=self.wrist_controller, pid_controller=self.pid_controller, slot=0, can_id=constants.k_wrist_motor_port,
                           pid_dict=constants.k_PID_dict_vel_wrist, pid_only=True, burn_flash=constants.k_burn_flash)

        # where are we when we start?  how do we stay closed w/o power?  do we leave pin in at power on?

        # set soft limits - do not let spark max put out power above/below a certain value
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limts)
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limts)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)

        # initialize the location of the wrist - say it's fully up at start
        self.sparkmax_encoder.setPosition(self.max_angle)
        self.angle = self.max_angle
        self.setpoint = self.angle
        SmartDashboard.putNumber('wrist_angle', self.angle)
        SmartDashboard.putNumber('wrist_setpoint', self.setpoint)

    def get_angle(self):  # getter for the relevant elevator parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.angle

    def set_wrist_angle(self, angle, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)
        self.angle = angle
        self.setpoint = angle
        SmartDashboard.putNumber('wrist_setpoint', angle)
        if wpilib.RobotBase.isSimulation():
            SmartDashboard.putNumber('wrist_angle', self.angle)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 25 == 0:
            self.angle = self.get_angle()
            SmartDashboard.putNumber('wrist_angle', self.angle)
