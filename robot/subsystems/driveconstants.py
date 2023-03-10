import math
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics

class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 1.2  # radians per second
    kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(26.5)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 11
    kRearLeftDrivingCanId = 13
    kFrontRightDrivingCanId = 15
    kRearRightDrivingCanId = 17

    kFrontLeftTurningCanId = 10
    kRearLeftTurningCanId = 12
    kFrontRightTurningCanId = 14
    kRearRightTurningCanId = 16

    kGyroReversed = False
