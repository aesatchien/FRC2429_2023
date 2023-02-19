"""
Wrist subsystem
Shares the following with networktables:  wrist_position
"""
from commands2 import SubsystemBase
import rev
import wpilib
import constants
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
        # 81x reduction motor to shaft, so in degrees it's 360./81.  Half a second at 6k rpm
        encoder_conversion_factor = 360./81
        self.sparkmax_encoder.setPositionConversionFactor(encoder_conversion_factor)  # mm per revolution
        self.pid_controller = self.wrist_controller.getPIDController()
        # where are we when we start?  how do we stay closed w/o power?  do we leave pin in at power on?

        # set soft limits - do not let spark max put out power above/below a certain value
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)

        # initialize the location of the wrist - say it's fully up at start
        self.sparkmax_encoder.setPosition(self.max_angle)

    def get_angle(self):  # getter for the relevant elevator parameter
        return self.sparkmax_encoder.getPosition()

    def set_wrist_angle(self, angle, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

