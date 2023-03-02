import math
import commands2
import rev
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain


class ChargeStationBalance(commands2.CommandBase):

    def __init__(self, container, drive: Drivetrain, velocity=20, tolerance=4) -> None:
        super().__init__()
        self.setName('ChargeStationBalance')
        self.container = container
        self.drive = drive
        self.velocity = velocity  # meters per MINUTE
        self.tolerance = tolerance  # degrees

        self.multipliers = [1, 1]

        self.addRequirements(drive)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.print_start_message()

    def execute(self) -> None:
        pitch = self.drive.navx.getPitch()

        if abs(pitch) > self.tolerance:
            # if robot is pitched downwards, drive backwards, or if robot is pitched upwards, drive forwards
            sign = math.copysign(1, pitch)
            for controller, multiplier in zip(self.drive.pid_controllers, self.multipliers):
                controller.setReference(sign * self.velocity * multiplier, rev.CANSparkMax.ControlType.kSmartVelocity, 2, arbFeedforward=1)
        else:
            [controller.setReference(0, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=2, arbFeedforward=0) for controller in self.drive.pid_controllers]

        self.drive.feed()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        [controller.setReference(0, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=1) for controller in self.drive.pid_controllers]

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")