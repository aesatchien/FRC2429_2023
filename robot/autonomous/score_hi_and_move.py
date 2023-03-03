import commands2
from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow
from autonomous.drive_move import DriveMove

class ScoreHiAndMove(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('ScoreHiAndMove')  # change this to something appropriate for this command
        self.container = container

        # Step 1.a
        # score hi from stow
        self.addCommands(ScoreHiConeFromStow(container=self.container))

        # Step 2.a
        # drive forward
        self.addCommands(DriveMove(container=self.container, drive=self.container.drive, setpoint=1, wait_to_finish=False))