import commands2
from wpilib import SmartDashboard


class TurretInitialize(commands2.CommandBase):

    def __init__(self, container, turret, samples=50) -> None:
        super().__init__()
        self.setName('Turret Initialize')
        self.container = container
        self.turret = turret
        self.samples = samples
        self.data = [0] * self.samples
        self.counter = 0

        self.addRequirements(self.turret)  # commandsv2 version of requirements

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.data = [0] * self.samples
        self.counter = 0
        SmartDashboard.putBoolean('turret_initialized', False)

    def execute(self) -> None:
        self.data[self.counter % self.samples] = self.turret.analog_abs_encoder.getDistance()
        self.counter += 1

    def isFinished(self) -> bool:
        return self.counter >= self.samples

    def end(self, interrupted: bool) -> None:

        average_encoder_value = sum(self.data) / self.samples
        self.turret.sparkmax_encoder.setPosition(average_encoder_value)
        print(f'set turret sparkmax encoder to {average_encoder_value}')

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putBoolean('turret_initialized', True)