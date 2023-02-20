"""
Arm subsystem
Shares the following with networktables:  arm_extension

Arm needs to
1) pull back all the way for stow
2) have a controllable distance from stow to fully extended
3) can probably trust the default encoder for the distance, just need to start with it retracted to define zero

"""

from commands2 import SubsystemBase
from wpilib import SmartDashboard
import rev
import constants
from misc.configure_controllers import configure_sparkmax

class Arm(SubsystemBase):
    def __init__(self):
        super().__init__()

        self.max_extension = 750  # need to see what is max legal amount
        self.min_extension = 0  # mm for now

        # initialize motors
        self.arm_controller = rev.CANSparkMax(constants.k_arm_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.arm_controller.setInverted(False)  # todo: need to check on this for the arm
        self.sparkmax_encoder = self.arm_controller.getEncoder()
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_arm_encoder_conversion_factor)  # mm per revolution
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_arm_encoder_conversion_factor)
        self.pid_controller = self.arm_controller.getPIDController()

        configure_sparkmax(sparkmax=self.arm_controller, pid_controller=self.pid_controller, slot=0, id=0,
                           pid_dict=constants.k_PID_dict_vel_arm, pid_only=True, burn_flash=constants.k_burn_flash)
        # where are we when we start?  how do we stay closed w/o power?  do we leave pin in at power on?

        # set soft limits - do not let spark max put out power above/below a certain value
        self.arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.arm_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_extension)
        self.arm_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_extension)

        # initialize the extension of the arm - say it's fully closed at start
        self.extension = 0
        self.sparkmax_encoder.setPosition(0)
        SmartDashboard.putNumber('arm_extension', self.extension)

    def get_extension(self):  # getter for the relevant elevator parameter
        return self.sparkmax_encoder.getPosition()

    def set_arm_extension(self, distance, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(distance, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(distance, rev.CANSparkMax.ControlType.kPosition)

        self.extension = distance
        SmartDashboard.putNumber('arm_extension', self.extension)


