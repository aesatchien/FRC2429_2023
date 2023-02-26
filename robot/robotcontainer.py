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
from subsystems.pneumatics import Pneumatics

from misc.axis_button import AxisButton
from commands.drive_by_joystick import DriveByJoystick
from commands.drive_velocity_stick import DriveByJoystickVelocity
from commands.turret_initialize import TurretInitialize
from commands.arm_move import ArmMove
from commands.turret_move import TurretMove
from commands.elevator_move import ElevatorMove
from commands.wrist_move import WristMove
from commands.manipulator_toggle import ManipulatorToggle
from commands.compressor_toggle import CompressorToggle
#from commands.elevator_drive import ElevatorDrive

from autonomous.score_from_stow import ScoreFromStow
from autonomous.upper_substation_pickup import UpperSubstationPickup


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
        self.pneumatics = Pneumatics()

        self.configureButtonBindings()

        # Set up default drive command
        if wpilib.RobotBase.isSimulation():
            self.drive.setDefaultCommand(DriveByJoystick(self, self.drive,lambda: -self.driver_controller.getRawAxis(1),
                    lambda: self.driver_controller.getRawAxis(4),))
        else:
            self.drive.setDefaultCommand(DriveByJoystickVelocity(container=self, drive=self.drive, control_type='velocity', scaling=1))

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

        # co-pilot controller
        self.co_driver_controller = wpilib.XboxController(constants.k_co_driver_controller_port)
        self.co_buttonLB = JoystickButton(self.co_driver_controller, 5)
        self.co_buttonRB = JoystickButton(self.co_driver_controller, 6)

        # testing turret and elevator
        enable_testing = True
        if enable_testing:
            self.buttonRight.whenPressed(TurretMove(self, self.turret, direction='up', wait_to_finish=True).withTimeout(2))
            self.buttonLeft.whenPressed(TurretMove(self, self.turret, direction='down', wait_to_finish=True).withTimeout(2))
            self.buttonDown.whenPressed(ElevatorMove(self, self.elevator, direction='up', wait_to_finish=True).withTimeout(1))
            self.buttonUp.whenPressed(ElevatorMove(self, self.elevator, direction='down', wait_to_finish=True).withTimeout(1))
            # manipulator
            self.buttonRB.whenPressed(ManipulatorToggle(container=self, pneumatics=self.pneumatics, force='open'))
            self.buttonLB.whenPressed(ManipulatorToggle(container=self, pneumatics=self.pneumatics, force='close'))

            #self.co_buttonRB.whileHeld(ElevatorDrive(container=self, elevator=self.elevator, button=self.co_buttonRB))

        # lots of putdatas for testing on the dash
        wpilib.SmartDashboard.putData(TurretInitialize(container=self, turret=self.turret))
        wpilib.SmartDashboard.putData(ScoreFromStow(container=self))
        wpilib.SmartDashboard.putData(UpperSubstationPickup(container=self))
        wpilib.SmartDashboard.putData(ManipulatorToggle(container=self, pneumatics=self.pneumatics))
        wpilib.SmartDashboard.putData(CompressorToggle(container=self, pneumatics=self.pneumatics))
        wpilib.SmartDashboard.putData(key='ElevatorMoveUp', data=ElevatorMove(container=self, elevator=self.elevator, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='ElevatorMoveDown', data=ElevatorMove(container=self, elevator=self.elevator, direction='down', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='WristMoveUp', data=WristMove(container=self, wrist=self.wrist, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='WristMoveDown', data=WristMove(container=self, wrist=self.wrist, direction='down', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='ArmMoveUp', data=ArmMove(container=self, arm=self.arm, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='ArmMoveDown', data=ArmMove(container=self, arm=self.arm, direction='down', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='TurretMoveUp', data=TurretMove(container=self, turret=self.turret, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='TurretMoveDown', data=TurretMove(container=self, turret=self.turret, direction='down', wait_to_finish=False))


        # commands2.button.JoystickButton(self.driverController, 3).whenHeld(
        #     HalveDriveSpeed(self.drive)
        # )