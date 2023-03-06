import math
import commands2
import rev
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain


class ChargeStationBalance(commands2.CommandBase):

    def __init__(self, container, drive: Drivetrain, velocity=60, tolerance=4, max_feed_forward=0.5, auto=False) -> None:
        super().__init__()
        self.setName('ChargeStationBalance')
        self.container = container
        self.drive = drive
        self.velocity = velocity  # meters per MINUTE
        self.tolerance = tolerance  # degrees
        self.max_feed_forward = max_feed_forward
        self.multipliers = [1, 1]
        self.count = 0
        self.auto = auto # are we in autonomous or teleop?

        # SmartDashboard.putNumber("ChargeStationBalance_arbFF", 1)

        self.addRequirements(drive)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.print_start_message()
        self.count = 0

    def execute(self) -> None:
        pitch = self.drive.navx.getPitch()
        self.count += 1
        if (self.count < 100 and self.auto is True): # in the initial second, let us climb faster
            speed_boost = 2
        elif (self.count > 100 and self.count < 150 and self.auto is True):
            speed_boost = 1.5  # in the initial second, let us climb faster
        else:
            speed_boost = 1

        if abs(pitch) > self.tolerance:
            # if robot is pitched downwards, drive backwards, or if robot is pitched upwards, drive forwards
            # use the value of the angle as a feedback parameter for feed forward
            sign = math.copysign(1, pitch)
            feed_forward = sign * abs(pitch) * (self.max_feed_forward/10)  # assume max V for 10 degrees
            feed_forward = min(self.max_feed_forward, feed_forward) if sign > 0 else max(-self.max_feed_forward, feed_forward)
            feed_forward *= speed_boost

            for controller, multiplier in zip(self.drive.pid_controllers, self.multipliers):
                controller.setReference(sign * self.velocity * multiplier * speed_boost, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=2,
                                        arbFeedforward=feed_forward)
        else:
            [controller.setReference(0, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=2, arbFeedforward=0) for controller in self.drive.pid_controllers]

        self.drive.feed()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        [controller.setReference(0, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=1) for controller in self.drive.pid_controllers]
        self.drive.spark_neo_left_front.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")