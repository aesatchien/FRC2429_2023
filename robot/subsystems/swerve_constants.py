import math
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from rev import CANSparkMax

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
    kFrontLeftChassisAngularOffset = 0 # -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = 0 # math.pi
    kBackRightChassisAngularOffset = 0 # math.pi / 2

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 21
    kRearLeftDrivingCanId = 23
    kFrontRightDrivingCanId = 25
    kRearRightDrivingCanId = 27

    kFrontLeftTurningCanId = 20
    kRearLeftTurningCanId = 22
    kFrontRightTurningCanId = 24
    kRearRightTurningCanId = 26

    kFrontLeftAbsEncoderPort = 0
    kFrontRightAbsEncoderPort = 1
    kBackLeftAbsEncoderPort = 2
    kBackRightAbsEncoderPort = 3

    kGyroReversed = False


class NeoMotorConstants:
    kFreeSpeedRpm = 5676

class ModuleConstants:
    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 14

    # Invert the turning encoder, since the output shaft rotates in the opposite direction of
    # the steering motor in the MAXSwerve Module.
    kTurningEncoderInverted = True # Change? Might not be true for 2429

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 4 * 0.0254  #  0.1016  =  four inches
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = 6.75  # From MK4i website, L2  #  From (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps  # CJH tested 3/19/2023, works ok  - 0.2235
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 0.25  #  CJH tested this 3/19/2023
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake
    kTurningMotorIdleMode = CANSparkMax.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp

class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )