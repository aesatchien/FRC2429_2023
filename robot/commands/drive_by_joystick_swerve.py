#  copying 1706's default swerve drive control

import math
import typing
import commands2
from subsystems.swerve import Swerve  # allows us to access the definitions
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d

class DriveByJoystickSwerve(commands2.CommandBase):
    def __init__(
        self, container,
        swerve: Swerve,
        field_oriented=False
    ) -> None:

        super().__init__()
        self.setName('drive_by_joystick_swerve')
        self.container = container
        self.swerve = swerve
        self.field_oriented = field_oriented

        self.addRequirements([self.swerve])

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:

        max_linear = 1  # m/s
        max_angular = 1  # rad/s
        # note that x is up/down on the left stick.  Don't want to invert x?
        desired_fwd = -self.input_transform(1.0*self.container.driver_controller.getRawAxis(1)) * max_linear
        desired_strafe = self.input_transform(1.0 * self.container.driver_controller.getRawAxis(0)) * max_linear
        desired_rot = -self.input_transform(1.0 * self.container.driver_controller.getRawAxis(4)) * max_angular

        correct_like_1706 = False  # this is what 1706 does, but Rev put all that in the swerve module's drive
        if correct_like_1706:
            desired_translation = Translation2d(desired_fwd, desired_strafe)
            desired_magnitude = desired_translation.norm()
            if desired_magnitude > max_linear:
                desired_translation = desired_translation * max_linear / desired_magnitude
            self.swerve.drive(desired_translation.X(), desired_translation.Y(), desired_rot,
                          fieldRelative=self.field_oriented, rateLimit=True)
        else:
            self.swerve.drive(xSpeed=desired_fwd,ySpeed=desired_strafe, rot=desired_rot,
                              fieldRelative=self.field_oriented, rateLimit=False)

    def end(self, interrupted: bool) -> None:
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rateLimit=True)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def apply_deadband(self, value, db_low=0.1, db_high=0.98):
        if abs(value) < db_low:
            return 0
        elif abs(value) > db_high:
            return 1 * math.copysign(1, value)
        else:
            return value

    def input_transform(self, value, a=0.9, b=0.1):
        db_value = self.apply_deadband(value)
        return a * db_value**3 + b * db_value