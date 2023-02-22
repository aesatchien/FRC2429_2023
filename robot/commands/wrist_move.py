import commands2
from wpilib import SmartDashboard
from subsystems.wrist import Wrist

class WristMove(commands2.CommandBase):

    def __init__(self, container, wrist:Wrist, setpoint, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('Wrist Move')
        self.container = container
        self.wrist = wrist
        self.setpoint = setpoint
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end

        self.addRequirements(self.wrist)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        # tell the elevator to go to position
        self.wrist.set_wrist_angle(angle=self.setpoint, mode='smartmotion')

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:  # wait for the wrist to get within x degrees
            return abs(self.wrist.get_angle() - self.setpoint) < 5
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.wrist.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")