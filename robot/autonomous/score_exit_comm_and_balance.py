import commands2

from autonomous.charge_station_balance import ChargeStationBalance
from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.drive_and_balance import DriveAndBalance


class ScoreExitCommAndBalance(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('DriveAndBalance')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(ScoreHiConeFromStow(self.container))

        self.addCommands(DriveSwerveAutoVelocity(container, container.drive, 1.5).withTimeout(1.5))

        self.addCommands(DriveSwerveAutoVelocity(container, container.drive, -1.75).withTimeout(0.5))

        self.addCommands(ChargeStationBalance(container, container.drive))
