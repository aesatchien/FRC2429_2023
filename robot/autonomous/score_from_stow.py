import commands2

import constants
from commands.elevator_move import ElevatorMove
from commands.turret_move import TurretMove

class ScoreFromStow(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('ScoreFromStow')  # change this to something appropriate for this command
        self.container = container

        # Step 1.a
        # raise the elevator , don't wait to end - can go concurrently with other moves
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,setpoint=980, wait_to_finish=False))

        # Step 1.b
        # Get turret into position - this will take 1.25s to get there but we move to next step anyway
        self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=180, wait_to_finish=False))

        # Step 2.a
        # lower wrist to upper scoring position 45°

        # Step 2.b
        # extend the arm fully

        # Optional - center turret on post

        # Step 4
        # Drop the wrist to level

        # Step 5
        # Open the manipulator

