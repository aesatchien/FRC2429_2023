import math
import commands2
from subsystems.drivetrain import Drivetrain
from wpilib import SmartDashboard


class ChargeStationBalance(commands2.CommandBase):

    def __init__(self, container, drive: Drivetrain, fwd) -> None:
        super().__init__()
        self.setName('ChargeStationBalance')
        self.container = container
        self.drive = drive

        self.fwd = fwd
        self.tolerance = 5

        self.addRequirements(drive)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pitch = self.drive.navx.getPitch()

        if abs(pitch) > self.tolerance:
            sign = math.copysign(pitch)

            """
            if bot is pitched downwards (-), drive backwards (-)
            if bot is pitched upwards (+), drive forwards (+)
            """

            self.drive.arcade_drive(sign * self.fwd, 0)
        else:
            self.drive.feed()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.drive.arcade_drive(0, 0)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")