import commands2
from subsystems.swerve_drivetrain_prototype import SwerveDriveTrain
import constants
from wpilib import SmartDashboard

class DriveWithSwerve(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive: SwerveDriveTrain) -> None:
        super().__init__()
        self.setName('Sample Name')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        self.drive.arcade_drive(self.container.driver_controller.getRawAxis(constants.k_controller_fwd_axis),
                                self.container.driver_controller.getRawAxis(constants.k_controller_strafe_axis),
                                self.container.driver_controller.getRawAxis(constants.k_controller_rot_axis))

    def isFinished(self) -> bool:
        return False # Just copied this off of drive_velocity_stick

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")