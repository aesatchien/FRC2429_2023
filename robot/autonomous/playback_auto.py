import commands2
import io
from wpilib import SmartDashboard
from subsystems.swerve import Swerve

class PlaybackAuto(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, swerve_log: io.TextIOWrapper, swerve: Swerve) -> None:
        super().__init__()
        self.setName('Playback Auto')  # change this to something appropriate for this command
        self.container = container
        self.swerve = swerve
        self.swerve_recording = [line.rstrip() for line in swerve_log]
        self.line_count = 0 

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        if self.line_count < len(self.swerve_recording):
            split_params = self.swerve_recording[self.line_count].split(', ')
            converted_params = [float(split_params[0]), float(split_params[1]), float(split_params[2]),
                        bool(split_params[3]), bool(split_params[4]), bool(split_params[5])]

            self.swerve.drive(*converted_params)

        self.line_count += 1

    def isFinished(self) -> bool:
        return self.line_count < len(self.swerve_recording)

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
