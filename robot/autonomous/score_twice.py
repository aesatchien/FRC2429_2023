import commands2

from autonomous.charge_station_balance import ChargeStationBalance
from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.drive_and_balance import DriveAndBalance
from autonomous.drive_wait import DriveWait

from commands.manipulator_toggle import ManipulatorToggle
from commands.wrist_move import WristMove


class ScoreTwice(commands2.CommandGroupBase): # Unfinished as of 4/6

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Score Twice')  # change this to something appropriate for this command

        self.container = container

        self.addCommands(ScoreHiConeFromStow(self.container)) 
        # elevator is down, turret is forwards, manipulator is open, arm is retracted, and wrist is in stow position

        self.addCommands(DriveSwerveAutoVelocity(self.container, self.container.drive, 1.5).withTimeout(1.5))

        self.addCommands(WristMove(self.container, self.contaienr.wrist, setpoint=container.wrist.positions['floor'], wait_to_finish=True))

        self.addCommands(ManipulatorToggle(self.container, self.container.pneumatics, force='close'))

        # Should we stow?

        self.addCommands()