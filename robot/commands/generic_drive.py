import commands2
import rev
from wpilib import SmartDashboard

class GenericDrive(commands2.CommandBase):

    def __init__(self, container, subsystem, max_velocity, axis, invert_axis = False) -> None:
        super().__init__()
        self.setName('Generic Drive')
        self.container = container
        self.subsystem = subsystem
        self.max_velocity = max_velocity
        self.axis = axis
        self.invert_axis = invert_axis

        self.velocity_multiplier = 0.5

        # Elevator, Turret, Wrist, and Arm all have pid_controller
        self.controller = self.subsystem.pid_controller

        self.addRequirements(self.subsystem)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        # get stick value and invert if necessary (if using Y-axis)
        stick = self.container.co_driver_controller.getRawAxis(self.axis)
        if self.invert_axis:
            stick *= -1

        velocity = stick * self.max_velocity * self.velocity_multiplier
        self.pid_controller.setReference(velocity, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=0)

    def isFinished(self) -> bool:
        return False  # this should be managed by the whileHeld()

    def end(self, interrupted: bool) -> None:
        self.pid_controller.setReference(0, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=0)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.elevator.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")