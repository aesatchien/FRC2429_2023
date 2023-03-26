"""
Scoring subsystem - are these better individually or all in one?
Shares the following with networktables:
arm_extension
elevator_height
turret_angle
wrist_position
manipulator_open
"""
from commands2 import SubsystemBase
import rev
import wpilib
from playingwithfusion import TimeOfFlight
import constants



class Scoring(SubsystemBase):
    def __init__(self):
        super().__init__()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.arm_controller = rev.CANSparkMax(constants.k_arm_motor_port, motor_type)
        self.elevator_controller = rev.CANSparkMax(constants.k_elevator_motor_port, motor_type)
        self.turret_controller = rev.CANSparkMax(constants.k_turret_motor_port, motor_type)
        self.wrist_controller = rev.CANSparkMax(constants.k_wrist_motor_port, motor_type)

        # set up distance sensor
        self.elevator_height_sensor = TimeOfFlight(constants.k_elevator_timeoflight)
        self.elevator_height_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)

        # should probably have positions that we need to map out
        self.elevator_positions = {'top':0, 'stow':0, 'upper_pickup':0, 'lower_pickup':0}
        self.wrist_positions = {'stow':0, 'score':0, 'flat':0, 'floor':0}
        self.turret_positions = {'stow':0, 'forward':0}
        self.arm_positions = {'stow':0, 'extended':0}


    def stow(self):
        """
        Is there a safe way to stow from any position?  e.g
        1) wrist up 2) arm retract 3) turret to back 3) elevator down
        Some of those can be concurrent
        """
        pass

    def unstow(self):
        """
        Is there a safe way to unstow from any position?  e.g
        1) elevator up 2) turret to front 3) arm extend  4) wrist level
        Some of those can be concurrent
        """
        pass



