"""
Turret subsystem
Shares the following with networktables:  turret_angle

Turret needs to:
1) know its ABSOLUTE position at all times  - we need a calibrate function
2) rotate on command
3) not exceed certain angular limit
4) not tip, so check speeds
5) go to certain certain angles here - for stow, and autonomous scoring
"""

from commands2 import SubsystemBase
import rev
import wpilib
from wpilib import SmartDashboard

import constants
from misc.configure_controllers import configure_controller
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Turret(SubsystemBase):
    def __init__(self):
        super().__init__()

        self.max_angle = 271
        self.min_angle = -45
        self.counter = 0

        # initialize motors
        self.turret_controller = rev.CANSparkMax(constants.k_turret_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.turret_controller.setInverted(True)  # true for turret
        self.sparkmax_encoder = self.turret_controller.getEncoder()
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_turret_encoder_conversion_factor)
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_turret_encoder_conversion_factor)  # needed for smartmotion
        self.pid_controller = self.turret_controller.getPIDController()
        configure_controller(sparkmax=self.turret_controller, pid_controller=self.pid_controller, slot=0, id=0,
                             pid_dict=constants.k_PID_dict_vel_turret, pid_only=True, burn_flash=constants.k_burn_flash)

        # set soft limits - do not let spark max put out power above/below a certain value
        self.turret_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.turret_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.turret_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
        self.turret_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)

        # same here, and need the turret encoder to be set to analog (jumper change)
        self.analog_abs_encoder = wpilib.AnalogEncoder(1)  # plug the analog encoder into channel 1
        self.analog_conversion_factor = 360.0  # 5V is 360 degrees
        self.analog_abs_encoder.setDistancePerRotation(self.analog_conversion_factor)

        # set the offset on the absolute analog encoder
        self.absolute_position_offset = 0.842  # this is what the absolute encoder reports when in stow position
        self.analog_abs_encoder.setPositionOffset(self.absolute_position_offset)  # now stow alignment is angle=0

        self.angle = self.get_angle()
        SmartDashboard.putNumber('turret_angle', self.angle)

    def get_angle(self):  # getter for the relevant turret parameter
        return self.sparkmax_encoder.getPosition()

    def set_turret_angle(self, angle, mode='smartmotion'):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

        self.angle = angle
        SmartDashboard.putNumber('turret_angle', self.angle)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 50 == 0:
            SmartDashboard.putNumber('turret_encoder', self.sparkmax_encoder.getPosition())
