import commands2
from wpilib import SmartDashboard
from subsystems.pneumatics import Pneumatics


class ManipulatorAutoGrab(commands2.CommandBase):

    def __init__(self, container, pneumatics: Pneumatics) -> None:
        super().__init__()
        self.setName('ManipulatorAutoGrab')
        self.container = container
        self.pneumatics = pneumatics
        self.has_game_piece = False
        self.addRequirements(pneumatics)

    def initialize(self) -> None:
        # ensure manipulator is open during initialization
        self.pneumatics.set_manipulator_piston(position='open')

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        target_distance = self.pneumatics.get_target_distance()
        grab_distance = 200 # mm

        if target_distance <= grab_distance and target_distance >= 50:
            self.pneumatics.set_manipulator_piston(position='close')
            self.has_game_piece = True

    def isFinished(self) -> bool:
        # whileHeld
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")