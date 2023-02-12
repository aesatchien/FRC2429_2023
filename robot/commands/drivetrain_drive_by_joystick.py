import typing
import commands2
from subsystems.drivesubsystem import DriveSubsystem


class DriveByJoystick(commands2.CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:
        super().__init__()

        self.drive = drive
        self.forward = forward
        self.rotation = rotation

        self.addRequirements([self.drive])

    def execute(self) -> None:
        self.drive.arcade_drive(self.forward(), self.rotation())