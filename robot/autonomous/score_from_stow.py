import commands2

import constants


class ScoreFromStow(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('ScoreFromStow')  # change this to something appropriate for this command
        self.container = container

        # Step 1.a
        # raise elevator to max height

        # Step 1.b
        # Get turret into position

        # Step 2.a
        # lower wrist to upper scoring position 45°

        # Step 2.b
        # extend the arm fully

        # Optional - center turret on post

        # Step 4
        # Drop the wrist to level

        # Step 5
        # Open the manipulator

