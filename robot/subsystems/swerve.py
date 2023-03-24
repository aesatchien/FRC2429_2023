import math
import typing
import wpilib

from commands2 import SubsystemBase
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry,)
import navx
import rev
from .swervemodule_2429 import SwerveModule
from .swerve_constants import DriveConstants


class Swerve (SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        # Create SwerveModules
        self.frontLeft = SwerveModule(
            drivingCANId=DriveConstants.kFrontLeftDrivingCanId, turningCANId=DriveConstants.kFrontLeftTurningCanId,
            encoder_analog_port=DriveConstants.kFrontLeftAbsEncoderPort,
            turning_encoder_offset=DriveConstants.k_lf_zero_offset, label='lf' )
        self.frontRight = SwerveModule(
            DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightAbsEncoderPort, DriveConstants.k_rf_zero_offset, label='rf')
        self.rearLeft = SwerveModule(
            DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftAbsEncoderPort, DriveConstants.k_lb_zero_offset, label='lb')
        self.rearRight = SwerveModule(
            DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightAbsEncoderPort, DriveConstants.k_rb_zero_offset, label='rb')

        # let's make this pythonic so we can do things quickly and with readability
        self.swerve_modules = [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]

        # The gyro sensor
        #self.gyro = wpilib.ADIS16470_IMU()
        self.gyro = navx.AHRS.create_spi()
        self.navx = self.gyro
        self.navx.zeroYaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
        self.gyro_calibrated = False

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation, self.currentTranslationDir, self.currentTranslationMag  = 0.0, 0.0, 0.0

        self.magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions())

    def periodic(self) -> None:
        # Update the odometry in the periodic block
        self.odometry.update(
            Rotation2d.fromDegrees(self.get_angle()),
            *self.get_module_positions(),)

        angles = [m.turningEncoder.getPosition() for m in self.swerve_modules]
        absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
        wpilib.SmartDashboard.putNumberArray(f'_angles', angles)
        wpilib.SmartDashboard.putNumberArray(f'_analog_radians', absolutes)
        wpilib.SmartDashboard.putNumber('_navx', self.get_angle())
        ypr = [self.navx.getYaw(), self.navx.getPitch(), self.navx.getRoll(), self.navx.getRotation2d().degrees()]
        wpilib.SmartDashboard.putNumberArray('_navx_YPR', ypr)

    def get_pose(self) -> Pose2d:
        # return the pose of the robot  TODO: update the dashboard here?
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.
        :param pose: The pose to which to set the odometry.
        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.get_angle()), pose, *self.get_module_positions())

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool, rate_limited: bool,) -> None:
        """Method to drive the robot using joystick info.
        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """

        if rate_limited:
            xSpeedCommanded = self.magLimiter.calculate(xSpeed)
            ySpeedCommanded = self.magLimiter.calculate(ySpeed)
            rotation_commanded = self.rotLimiter.calculate(rot)
        else:
            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed
            rotation_commanded = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = rotation_commanded * DriveConstants.kMaxAngularSpeed

        wpilib.SmartDashboard.putNumberArray('_xyr', [xSpeedDelivered, ySpeedDelivered, rotDelivered])

        # create the swerve state array depending on if we are field relative or not
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(self.get_angle()),)
            if fieldRelative else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered))

        # normalize wheel speeds so we do not exceed our speed limit
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)

    def drive_forwards_vel(self, targetvel, pidSlot=0, l_feed_forward=0, r_feed_forward=0):
        feeds = [l_feed_forward, r_feed_forward, l_feed_forward, r_feed_forward]
        control_type = rev.CANSparkMax.ControlType.kSmartVelocity
        self.setModuleStates([SwerveModuleState(0, Rotation2d.fromDegrees(0))]*4) # Turn the swerve into a tank drive
        for feed, module in zip (feeds, self.swerve_modules):
            module.drivingPIDController.setReference(targetvel, control_type, pidSlot=pidSlot, arbFeedforward=feed)

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            self.setX()

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        angles = [4, -45, -45, 45]
        for angle, swerve_module in zip(angles, self.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(angle)))

    def setModuleStates(self, desiredStates: typing.Tuple[SwerveModuleState]) -> None:
        """Sets the swerve ModuleStates.
        :param desiredStates: The desired SwerveModule states.
        """
        desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxTotalSpeed)
        for idx, m in enumerate(self.swerve_modules):
            m.setDesiredState(desiredStates[idx])

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        [m.resetEncoders() for m in self.swerve_modules]

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self) -> float:
        """Returns the heading of the robot.
        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.get_angle()).getDegrees()

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.
        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if DriveConstants.kGyroReversed else 1.0)

    def get_module_positions(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getPosition() for m in self.swerve_modules]

    def get_angle(self):
        return -self.gyro.getAngle() if DriveConstants.kGyroReversed else self.gyro.getAngle()
