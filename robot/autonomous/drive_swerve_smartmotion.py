import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from subsystems.swerve import Swerve


class DriveSwerveSmartmotion(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive: Swerve, setpoint) -> None:
        super().__init__()
        self.setName('Drive Swerve Smartmotion')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.addRequirements(self.drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        if False:
            for module in self.drive.swerve_modules:
                module.drivingEncoder.setPosition(0)
            self.drive.setModuleStates([SwerveModuleState(0, Rotation2d.fromDegrees(0))]*4) # Turn the swerve into a tank drive
            self.drive.set_drive_motor_references(self.setpoint)

    def execute(self) -> None:
        self.drive.drive(1/4, 0, 0, False, False)
        if False:
            for module in self.drive.swerve_modules:
                module.turning_PID_controller.setSetpoint(0)
            self.drive.set_drive_motor_references(self.setpoint)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")