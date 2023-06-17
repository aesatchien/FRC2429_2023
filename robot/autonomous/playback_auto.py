import commands2
import io
import json
from wpilib import SmartDashboard
from subsystems.swerve import Swerve

class PlaybackAuto(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, input_log_path) -> None:
        super().__init__()
        self.setName('Playback Auto')  # change this to something appropriate for this command
        self.container = container
        with open(input_log_path, 'r') as input_json:
            self.input_log = json.load(input_json)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        self.line_count = 0

    def execute(self) -> None:
        current_inputs = self.input_log[self.line_count]
        self.container.drive.drive(current_inputs['driver_controller']['axis']['axis0'],
                                   current_inputs['driver_controller']['axis']['axis1'],
                                   current_inputs['driver_controller']['axis']['axis4'])

        self.line_count += 1

    def isFinished(self) -> bool:
        return self.line_count < len(self.input_log)

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
