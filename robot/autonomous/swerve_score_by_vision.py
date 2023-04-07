import commands2
from wpilib import SmartDashboard

from subsystems.turret import Turret
from subsystems.swerve import Swerve
from subsystems.arm import Arm
from subsystems.vision import Vision
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.pneumatics import Pneumatics
from commands.elevator_move import ElevatorMove
from commands.arm_move import ArmMove
from commands.wrist_move import WristMove
from commands.manipulator_toggle import ManipulatorToggle
from .drive_swerve_auto_velocity import DriveSwerveAutoVelocity

class SwerveScoreByVision(commands2.CommandBase):
    def __init__(self, container, swerve: Swerve, elevator: Elevator, turret: Turret, arm: Arm, vision: Vision, wrist: Wrist, pneumatics: Pneumatics):
        super().__init__()
        self.setName('SwerveScoreByVision')
        self.container = container
        self.swerve = swerve
        self.elevator = elevator
        self.turret = turret
        self.arm = arm
        self.vision = vision
        self.wrist = wrist
        self.pneumatics = pneumatics
        self.finished = False # idk if we need this
        # self.addRequirements()
    def initialize(self) -> None:

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        # TODO: make sure we see tags and, if we're holding a cone, retroreflective tape

        # strafe
        holding_cube = False # TODO: get this value
        if holding_cube:
            commands2.ScheduleCommand(DriveSwerveAutoVelocity(self.container, self.swerve, 1, direction='strafe').withTimeout(self.vision.get_tag_strafe()))
        else:
            print(f'Strafing {self.vision.get_green_strafe()} meters (I think)')
            commands2.ScheduleCommand(DriveSwerveAutoVelocity(self.container, self.swerve, 1, direction='strafe').withTimeout(self.vision.get_green_strafe()))

        # drive forwards, setup arm and elevator
        if self.turret.get_angle() > 270 or self.turret.get_angle() < 90: # we're scoring on mid
            # add  (dist from tag to center of mid nodes) - (length between camera & end of arm when arm middley extended)
            # to drive_time in meters
            drive_time = self.vision.camera_dict['tags']['distance_entry'].get() + 0 # in meters. Using apriltags for more accuracy
            commands2.ScheduleCommand(ElevatorMove(self.container, self.elevator, setpoint=self.elevator.positions['low'], wait_to_finish=False))
            commands2.ScheduleCommand(ArmMove(self.container, self.arm, setpoint=self.arm.positions['middle'], wait_to_finish=False))
            commands2.ScheduleCommand(DriveSwerveAutoVelocity(self.container, self.swerve, 1).withTimeout(drive_time)) # go 1 m/s for drive_time seconds
        else: # we're scoring on hi
            # add (dist from tag to center of hi nodes) - (length between camera & end of arm when arm fully extended)
            # to drive_time in meters
            drive_time = self.vision.camera_dict['tags']['distance_entry'].get() + 0 # in meters
            commands2.ScheduleCommand(ElevatorMove(self.container, self.elevator, setpoint=self.elevator.positions['top'], wait_to_finish=False))
            commands2.ScheduleCommand(ArmMove(self.container, self.arm, setpoint=self.arm.positions['full'], wait_to_finish=False))
            commands2.ScheduleCommand(DriveSwerveAutoVelocity(self.container, self.swerve, 1).withTimeout(drive_time)) # go 1 m/s for drive_time seconds
        print(f'Driving {drive_time} meters (I think)')
        
        # drop
        commands2.ScheduleCommand(WristMove(self.container, self.wrist, setpoint=Wrist.positions['flat'], wait_to_finish=True))
        commands2.ScheduleCommand(ManipulatorToggle(self.container, self.pneumatics, force='open'))
        self.finished = True

    def execute(self) -> None:
        pass
    
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")