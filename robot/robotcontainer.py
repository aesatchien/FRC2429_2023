import wpilib
from wpilib.interfaces import GenericHID

import commands2
from commands2.button import JoystickButton, POVButton
import time
import constants

from subsystems.drivetrain import Drivetrain
from subsystems.arm import Arm
from subsystems.wrist import Wrist
from subsystems.elevator import Elevator
from subsystems.turret import Turret

from misc.axis_button import AxisButton
from commands.drive_by_joystick import DriveByJoystick
from commands.turret_initialize import TurretInitialize
from commands.turret_move import TurretMove
from commands.elevator_move import ElevatorMove


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.start_time = time.time()

        # The robot's subsystems
        self.drive = Drivetrain()
        self.turret = Turret()
        self.arm = Arm()
        self.wrist = Wrist()
        self.elevator = Elevator()

        self.configureButtonBindings()

        # set up default drive command
        self.drive.setDefaultCommand(
            DriveByJoystick(self, self.drive,
                lambda: -self.driver_controller.getRawAxis(1),
                lambda: self.driver_controller.getRawAxis(4),
            )
        )

        # initialize the turret
        commands2.ScheduleCommand(TurretInitialize(container=self, turret=self.turret, samples=50)).initialize()

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # The driver's controller
        self.driver_controller = wpilib.XboxController(constants.k_driver_controller_port)
        self.buttonA = JoystickButton(self.driver_controller, 1)
        self.buttonB = JoystickButton(self.driver_controller, 2)
        self.buttonX = JoystickButton(self.driver_controller, 3)
        self.buttonY = JoystickButton(self.driver_controller, 4)
        self.buttonLB = JoystickButton(self.driver_controller, 5)
        self.buttonRB = JoystickButton(self.driver_controller, 6)
        self.buttonBack = JoystickButton(self.driver_controller, 7)
        self.buttonStart = JoystickButton(self.driver_controller, 8)
        self.buttonUp = POVButton(self.driver_controller, 0)
        self.buttonDown = POVButton(self.driver_controller, 180)
        self.buttonLeft = POVButton(self.driver_controller, 270)
        self.buttonRight = POVButton(self.driver_controller, 90)
        self.buttonLeftAxis = AxisButton(self.driver_controller, 2)
        self.buttonRightAxis = AxisButton(self.driver_controller, 3)

        # testing turret and elevator
        enable_testing = False
        if enable_testing:
            self.buttonRight.whenPressed(TurretMove(self, self.turret, setpoint=10, wait_to_finish=True).withTimeout(2))
            self.buttonLeft.whenPressed(TurretMove(self, self.turret, setpoint=-10, wait_to_finish=True).withTimeout(2))
            self.buttonDown.whenPressed(ElevatorMove(self, self.elevator, setpoint=800, wait_to_finish=True).withTimeout(1))
            self.buttonUp.whenPressed(ElevatorMove(self, self.elevator, setpoint=200, wait_to_finish=True).withTimeout(1))


        # commands2.button.JoystickButton(self.driverController, 3).whenHeld(
        #     HalveDriveSpeed(self.drive)
        # )