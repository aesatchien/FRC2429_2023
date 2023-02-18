"""
Elevator subsystem
Shares the following with networktables:  elevator_height
Elevator needs to:
1) know its vertical position
2) not exceed set min/max values
3) drive to any of a element from a list of positions
4) allow a driver override of position
5) needs a function to set and hold a position



"""
from commands2 import SubsystemBase
import rev
from playingwithfusion import TimeOfFlight

import constants
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Elevator(SubsystemBase):
    def __init__(self):
        super().__init__()

        self.max_height = 900  # the bottom of the carriage is 39in (991mm)  above the bottom at max height
        self.min_height = 100  # mm for now

        # initialize motors
        self.elevator_controller = rev.CANSparkMax(constants.k_elevator_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.sparkmax_encoder = self.elevator_controller.getEncoder()
        encoder_conversion_factor = 0.253 * 25.4   # 16x reduction motor to shaft, one sprocket rot is 4.05in so 0.253
        self.sparkmax_encoder.setPositionConversionFactor(encoder_conversion_factor)  # mm per revolution
        self.pid_controller = self.elevator_controller.getPIDController()

        # set up distance sensor
        self.elevator_height_sensor = TimeOfFlight(constants.k_elevator_timeoflight)
        self.elevator_height_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)

        # elevator should probably have positions that we need to map out
        self.positions = {'top':980, 'bottom':20, 'upper_pickup':600, 'lower_pickup':300}

        # set soft limits - do not let spark max put out power above/below a certain value
        self.elevator_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.elevator_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.elevator_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_height)
        self.elevator_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_height)

        # initialize the height of the elevator  - sensor is in mm, so stick with that
        initial_position = self.elevator_height_sensor.getRange()
        self.sparkmax_encoder.setPosition(initial_position)

    def get_height(self):  # getter for the relevant elevator parameter
        return self.sparkmax_encoder.getPosition()

    def set_elevator_height(self, height, mode='smartmotion'):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(height, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(height, rev.CANSparkMax.ControlType.kPosition)
