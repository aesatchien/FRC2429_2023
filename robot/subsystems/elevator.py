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

        # initialize motors
        self.elevator_controller = rev.CANSparkMax(constants.k_elevator_motor_port, rev.CANSparkMax.MotorType.kBrushless)

        # set up distance sensor
        self.elevator_height_sensor = TimeOfFlight(constants.k_elevator_timeoflight)
        self.elevator_height_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)

        # elevator should probably have positions that we need to map out
        self.positions = {'top':0, 'bottom':0, 'upper_pickup':0, 'lower_pickup':0}


