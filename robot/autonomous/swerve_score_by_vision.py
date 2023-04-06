import commands2

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

class SwerveScoreByVision(commands2.SequentialCommandGroup):
    def __init__(self, container, swerve: Swerve, elevator: Elevator, turret: Turret, arm: Arm, vision: Vision, wrist: Wrist, pneumatics: Pneumatics):
        super().__init__()
        self.setName('SwerveScoreByVision')
        # TODO: make sure we see tags and, if we're holding a cone, retroreflective tape

        # strafe
        holding_cube = False # TODO: get this value
        if holding_cube:
            self.addCommands(DriveSwerveAutoVelocity(container, swerve, 1, direction='strafe').withTimeout(vision.get_tag_strafe()))
        else:
            print(f'Strafing {vision.get_green_strafe} meters (I think)')
            self.addCommands(DriveSwerveAutoVelocity(container, swerve, 1, direction='strafe').withTimeout(vision.get_green_strafe()))

        # drive forwards, setup arm and elevator
        if turret.get_angle() > 270 or turret.get_angle() < 90: # we're scoring on mid
            # add  (dist from tag to center of mid nodes) - (length between camera & end of arm when arm middley extended)
            # to drive_time in meters
            drive_time = vision.camera_dict['tags']['distance_entry'].getDouble(0) + 0 # in meters. Using apriltags for more accuracy
            self.addCommands(ElevatorMove(container, elevator, setpoint=elevator.positions['low'], wait_to_finish=False))
            self.addCommands(ArmMove(container, arm, setpoint=arm.positions['middle'], wait_to_finish=False))
            self.addCommands(DriveSwerveAutoVelocity(container, swerve, 1).withTimeout(drive_time)) # go 1 m/s for drive_time seconds
        else: # we're scoring on hi
            # add (dist from tag to center of hi nodes) - (length between camera & end of arm when arm fully extended)
            # to drive_time in meters
            drive_time = vision.camera_dict['tags']['distance_entry'].getDouble(0) + 0 # in meters
            self.addCommands(ElevatorMove(container, elevator, setpoint=elevator.positions['top'], wait_to_finish=False))
            self.addCommands(ArmMove(container, arm, setpoint=arm.positions['full'], wait_to_finish=False))
            self.addCommands(DriveSwerveAutoVelocity(container, swerve, 1).withTimeout(drive_time)) # go 1 m/s for drive_time seconds
        print(f'Driving {drive_time} meters (I think)')
        
        # drop
        self.addCommands(WristMove(container, wrist, setpoint=Wrist.positions['flat'], wait_to_finish=True))
        self.addCommands(ManipulatorToggle(container, pneumatics, force='open'))
