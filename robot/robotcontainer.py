import wpilib
from wpilib.interfaces import GenericHID

import commands2
import commands2.button
import time
import constants
from wpilib import Timer

from subsystems.drivetrain import Drivetrain
from subsystems.arm import Arm
from subsystems.wrist import Wrist
from subsystems.elevator import Elevator
from subsystems.turret import Turret

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
        self.drive = Drivetrain()

        # self.configureButtonBindings()

        # set up default drive command
        self.drive.setDefaultCommand(
            DriveByJoystick(self, self.drive,
                lambda: -self.driver_controller.getRawAxis(1),
                lambda: self.driver_controller.getRawAxis(4),
            )
        )


    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return Timer.getMatchTime()

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