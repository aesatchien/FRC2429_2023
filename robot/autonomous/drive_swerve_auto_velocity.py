import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc


class DriveSwerveAutoVelocity(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive: Swerve, velocity, direction='forwards') -> None:
        super().__init__()
        self.setName('DriveSwerveAutoVelocity')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.addRequirements(self.drive)  # commandsv2 version of requirements
        self.setpoint_velocity = velocity  # in m/s, gets normalized when sent to drive
        self.direction = direction

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
        # drive at the velocity passed to the function
        if self.direction == 'forwards':
            self.drive.drive(self.setpoint_velocity / dc.kMaxSpeedMetersPerSecond, 0, 0, False, False)
        elif self.direction == 'strafe':
            self.drive.drive(0, self.setpoint_velocity / dc.kMaxSpeedMetersPerSecond, 0, False, False)

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