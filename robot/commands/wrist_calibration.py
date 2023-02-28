import commands2
from wpilib import SmartDashboard
import rev
from subsystems.wrist import Wrist


class WristCalibration(commands2.CommandBase):

    def __init__(self, container, wrist:Wrist, velocity=2500) -> None:
        super().__init__()
        self.setName('WristCalibration')
        self.container = container
        self.wrist = wrist
        self.velocity = velocity
        self.addRequirements(self.wrist)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        # turn off the reverse limit
        self.wrist.wrist_controller.clearFaults()
        self.wrist.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, False)
        self.wrist.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, False)
        self.wrist.pid_controller.setReference(4, rev.CANSparkMax.ControlType.kVoltage)  # should do velocity?

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        # self.wrist.pid_controller.setReference(-1000, rev.CANSparkMax.ControlType.kVelocity)
        SmartDashboard.putNumber("WristCurrent", self.wrist.wrist_controller.getOutputCurrent())
        SmartDashboard.putNumber("WristVoltage", self.wrist.wrist_controller.getAppliedOutput())

    def isFinished(self) -> bool:
        # Stop when we hit the limit switch or too much current
        return self.wrist.forward_limit_switch.get()  or self.wrist.wrist_controller.getOutputCurrent() > 10

    def end(self, interrupted: bool) -> None:
        self.wrist.set_encoder_position(angle=self.wrist.max_angle)  # now we have a new maximum
        self.wrist.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.wrist.pid_controller.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.wrist.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")