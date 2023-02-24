import commands2
from wpilib import SmartDashboard
from subsystems.arm import Arm

class ArmMove(commands2.CommandBase):

    def __init__(self, container, arm:Arm, setpoint=None, direction=None, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('Arm Move')
        self.container = container
        self.arm = arm
        self.setpoint = setpoint
        self.direction = direction
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end

        self.addRequirements(self.arm)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        # tell the arm to go to position
        position = self.arm.get_extension()
        # tell the elevator to go to position
        if self.setpoint is None:
            if self.direction == 'up':
                allowed_positions = [x for x in sorted(self.arm.positions.values()) if x > position]
                print(allowed_positions)
                temp_setpoint = sorted(allowed_positions)[0] if len(allowed_positions) > 0 else position
            else:
                allowed_positions = [x for x in sorted(self.arm.positions.values()) if x < position]
                print(allowed_positions)
                temp_setpoint = sorted(allowed_positions)[-1] if len(allowed_positions) > 0 else position

            self.arm.set_arm_extension(distance=temp_setpoint, mode='smartmotion')
            print(f'Setting arm from {position:.0f} to {temp_setpoint}')
        else:
            self.arm.set_arm_extension(distance=self.setpoint, mode='smartmotion')
            print(f'Setting arm from {position:.0f} to {self.setpoint}')

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:  # wait for the arm to get within x mm
            return abs(self.arm.get_extension() - self.setpoint) < 25
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.arm.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")