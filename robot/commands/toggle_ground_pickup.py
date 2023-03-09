import commands2
from wpilib import SmartDashboard

from subsystems.pneumatics import Pneumatics
from subsystems.wrist import Wrist

class ToggleGroundPickup(commands2.CommandBase):
    def __init__(self, container, pneumatics: Pneumatics, wrist: Wrist) -> None:
        super().__init__()
        self.setName('ToggleGroundPickup')

        self.container = container
        self.penumatics = pneumatics
        self.wrist = wrist
        
        self.has_game_piece = False

    def initialize(self) -> None:
        # open manipulator
        self.pneumatics.set_manipulator_piston(position='open')
        
        # deploy wrist
        self.wrist.set_wrist_angle(angle=self.wrist.positions['floor'])

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        pass
        # if self.pneumatics.
    def end(self, interrupted: bool) -> None:
        # stow wrist


        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f'** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **')

    def print_start_message(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
