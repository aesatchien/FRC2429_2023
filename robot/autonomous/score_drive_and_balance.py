import commands2

import constants

from autonomous.charge_station_balance import ChargeStationBalance
from autonomous import drive_robot
from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow


class ScoreDriveAndBalance(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('DriveAndBalance')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(ScoreHiConeFromStow(self.container))

        # drive onto the charge station
        # self.addCommands(DriveMove(container=self.container, drive=self.container.drive,
        #                               setpoint=1, wait_to_finish=True).withTimeout(5))
        self.addCommands(drive_robot.get_command_for_driving_meters(self.container, 1.5))

        # maintain balance
        self.addCommands(ChargeStationBalance(container=self.container))




