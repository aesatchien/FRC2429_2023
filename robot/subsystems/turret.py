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

import constants
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim



class Turret(SubsystemBase):
    def __init__(self):
        super().__init__()

        self.max_angle = 180
        self.min_angle = -180

        # initialize motors
        self.turret_controller = rev.CANSparkMax(constants.k_turret_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.default_encoder = self.turret_controller.getEncoder()
        self.default_encoder_conversion_factor = 360 / 462.0  # Armabot has 462:1 gear ratio?  Circle has 360 degrees.
        self.default_encoder.setPositionConversionFactor(self.default_encoder_conversion_factor)

        # same here, and need the turret encoder to be set to analog (jumper change)
        self.analog_absolute_encoder = wpilib.AnalogEncoder(1)  # plug the analog encoder into channel 1
        self.analog_conversion_factor = 360.0  # 5V is 360 degrees
        self.analog_absolute_encoder.setDistancePerRotation(self.analog_conversion_factor)


        # current_angle = self.analog_absolute_encoder.getDistance()
        # self.default_encoder.setPosition(current_angle)

    def rotate_to_angle(self, angle):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """
        pass
