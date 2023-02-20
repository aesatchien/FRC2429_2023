"""
Wrist subsystem
Shares the following with networktables:  wrist_position
"""
from commands2 import SubsystemBase
import rev
from wpilib import SmartDashboard
import constants
from misc.configure_controllers import configure_sparkmax
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Wrist(SubsystemBase):
    # wrist should probably have four positions that we need to map out
    positions = {'stow': 125, 'score': 45, 'flat': 0, 'floor': -30}

    def __init__(self):
        super().__init__()

        # defining angles so 0 is horizontal
        self.max_angle = 125  # call all the way up 125 degrees  todo: remeasure
        self.min_angle = -30

        # initialize motors
        self.wrist_controller = rev.CANSparkMax(constants.k_wrist_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.wrist_controller.setInverted(False)  # todo need to check on this for the wrist
        self.sparkmax_encoder = self.wrist_controller.getEncoder()

        # update sparkmax with appropriate system gains and constraints
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_wrist_encoder_conversion_factor)  # mm per revolution
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_wrist_encoder_conversion_factor)  # necessary for smartmotion to behave
        self.pid_controller = self.wrist_controller.getPIDController()
        configure_sparkmax(sparkmax=self.wrist_controller, pid_controller=self.pid_controller, slot=0, id=0,
                           pid_dict=constants.k_PID_dict_vel_wrist, pid_only=True, burn_flash=constants.k_burn_flash)

        # where are we when we start?  how do we stay closed w/o power?  do we leave pin in at power on?

        # set soft limits - do not let spark max put out power above/below a certain value
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)

        # initialize the location of the wrist - say it's fully up at start
        self.sparkmax_encoder.setPosition(self.max_angle)
        self.angle = self.max_angle
        SmartDashboard.putNumber('wrist_angle', self.angle)

    def get_angle(self):  # getter for the relevant elevator parameter
        return self.sparkmax_encoder.getPosition()

    def set_wrist_angle(self, angle, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)
        self.angle = angle
        SmartDashboard.putNumber('wrist_angle', angle)

