import commands2
import os
import json
import math
from wpilib import SmartDashboard
from subsystems.swerve_constants import DriveConstants
from commands.manipulator_toggle import ManipulatorToggle
import constants

class PlaybackAuto(commands2.CommandBase):
    # look, ma, no path planning!

    def __init__(self, container, input_log_path) -> None:
        super().__init__()
        self.setName('Playback Auto')
        self.container = container
        self.input_log_path = input_log_path
        self.addRequirements(self.container.drive)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        with open(self.input_log_path, 'r') as input_json:
            self.input_log = json.load(input_json)

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        self.line_count = 1

    def execute(self) -> None:
        current_inputs = self.input_log[self.line_count]
        previous_inputs = self.input_log[self.line_count-1]

        if current_inputs['driver_controller']['button']['LB']:
            slowmode_multiplier = constants.k_slowmode_multiplier
        elif current_inputs['driver_controller']['axis']['axis2']:
            slowmode_multiplier = 1.5 * constants.k_slowmode_multiplier
        else: 
            slowmode_multiplier = 1

        print('\n FWD: ' + str(-self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis1'])))
        print('STRAFE: ' + str(self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis0'])))
        print('ROT: ' + str(-self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis4'])) + '\n')

        self.container.drive.drive(-self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis1']),
                                   self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis0']),
                                   -self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis4']),
                                   fieldRelative=True, rate_limited=False, keep_angle=True)
        
        # Get only rising edges. Not sure this is necessary
        if current_inputs['driver_controller']['button']['POV'] == 180 and not previous_inputs['driver_controller']['button']['POV'] == 180:
            print("* * ! TOGGLING MANIPULATOR ! * *")
            commands2.ScheduleCommand(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics))

        self.line_count += 1

    # stuff from drive_by_joystick_swerve- might it be better and DRYer to have it in the subsystem?
    def apply_deadband(self, value, db_low=DriveConstants.k_inner_deadband, db_high=DriveConstants.k_outer_deadband):
        if abs(value) < db_low:
            return 0
        elif abs(value) > db_high:
            return 1 * math.copysign(1, value)
        else:
            return value

    def input_transform(self, value, a=0.9, b=0.1):
        db_value = self.apply_deadband(value)
        return a * db_value**3 + b * db_value

    def isFinished(self) -> bool:
        return self.line_count >= len(self.input_log)

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
