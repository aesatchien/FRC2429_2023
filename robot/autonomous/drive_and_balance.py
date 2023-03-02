import commands2

import constants

from autonomous.drive_move import DriveMove
from autonomous.charge_station_balance import ChargeStationBalance


class DriveAndBalance(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('DriveAndBalance')  # change this to something appropriate for this command
        self.container = container

        # drive onto the charge station
        self.addCommands(DriveMove(container=self.container, drive=self.container.drive,
                                      setpoint=1, wait_to_finish=True).withTimeout(5))

        # maintain balance
        self.addCommands(ChargeStationBalance(container=self.container, drive=self.container.drive).withTimeout(5))




