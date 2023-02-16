"""
Elevator subsystem
Shares the following with networktables:  elevator_height
"""
from commands2 import SubsystemBase
import rev
from playingwithfusion import TimeOfFlight

import constants
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Elevator(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        self.elevator_controller = rev.CANSparkMax(constants.k_elevator_motor_port, rev.CANSparkMax.MotorType.kBrushless)

        # det up distance sensor
        self.distance_sensor = TimeOfFlight(constants.k_elevator_timeoflight)
        self.distance_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)

        # elevator should probably have positions that we need to map out
        self.positions = {'top':0, 'bottom':0, 'upper_pickup':0, 'lower_pickup':0}


