'''returns a command for a SequentialCommandGroup to run'''
import commands2
import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from subsystems.drivetrain import Drivetrain
from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants, AutoConstants
from .drive_tank import DriveTank


def get_command_for_driving_meters(container, setpoint, wait_to_finish=True):
    '''Returns a command that drives the container's drive setpoint meters. Works with Swerve and Drivetrain.'''
    if isinstance(container.drive, Drivetrain):
        return DriveTank(container=container, drive=container.drive, setpoint=setpoint, wait_to_finish=wait_to_finish)
    elif isinstance(container.drive, Swerve):
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction and end setpoint meters straight ahead of where we started, facing forward
            [Pose2d(0, 0, Rotation2d(0)), Pose2d(setpoint, 0, Rotation2d(0))],
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        swerveControllerCommand = commands2.Swerve4ControllerCommand(
            trajectory,
            container.drive.get_pose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            container.drive.setModuleStates,
            [container.drive],
        )

        # Reset odometry to the starting pose of the trajectory.
        container.drive.resetOdometry(trajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            lambda: container.drive.drive(0, 0, 0, False, False)
        )  
