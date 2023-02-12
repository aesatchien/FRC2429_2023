import wpilib
from wpilib.interfaces import GenericHID

import commands2
import commands2.button

import constants

from subsystems.drivesubsystem import DriveSubsystem
from commands.drivetrain_drive_by_joystick import DriveByJoystick


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The driver's controller
        self.driver_controller = wpilib.XboxController(constants.k_driver_controller_port)

        # The robot's subsystems
        self.drive = DriveSubsystem()

        # self.configureButtonBindings()

        # set up default drive command
        self.drive.setDefaultCommand(
            DriveByJoystick(
                self.drive,
                lambda: -self.driver_controller.getRawAxis(1),
                lambda: self.driver_controller.getRawAxis(4),
            )
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        pass

        # commands2.button.JoystickButton(self.driverController, 3).whenHeld(
        #     HalveDriveSpeed(self.drive)
        # )