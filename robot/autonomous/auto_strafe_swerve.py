import commands2
import wpilib
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d, Pose2d
from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc
import math
from subsystems.vision import Vision

class AutoStrafeSwerve(commands2.CommandBase):

    def __init__(self, container, drive:Swerve, vision:Vision, target_distance=0.5, target_type=None, auto=True) -> None:
        super().__init__()
        self.setName('AutoStrafeSwerve')
        self.container = container
        self.vision = vision
        self.drive = drive
        self.target_distance = target_distance  # note this must be updated in initialize, not here
        self.target_type = target_type
        self.addRequirements(container.drive)

        # Set PID controller so that 0.5m degrees will cause maximum output of 1.  Max speeds will be handled by time.
        self.strafe_controller = PIDController(1, 0, 0)  # note this does not clamp to ±1 unless you do it yourself
        self.strafe_controller.setTolerance(0.05)  # a few centimeters

        self.auto = auto  # allows for operator to use with minimum velocity

        # should we set up a velocity transition, velocities in m/s
        self.strafe_start_time = 0  # do not confuse with the start time status message
        self.max_velocity = 2  # in m/s, so do not forget to normalize when sent to drive function
        self.min_velocity = 1
        self.decay_rate = 10 #  20 transitions in about 0.25s, 10 is about 0.5 s to transition from high to low
        self.transition_time_center = 0.8  # center time of our transition, in seconds

        self.start_pose = Pose2d()

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.strafe_start_time = wpilib.Timer.getFPGATimestamp()
        self.print_start_message()

        if wpilib.RobotBase.isReal():
            self.start_pose = self.drive.get_pose(report=True)
        else:  # don't have swerve pose working in the sim yet, so fake it
            sim_pose = wpilib.SmartDashboard.getNumberArray('drive_pose', [0, 0, 0])
            self.start_pose = Pose2d(sim_pose[0], sim_pose[1], Rotation2d(sim_pose[2]) )

        if self.target_type == 'tag':
            self.target_distance = self.vision.get_tag_strafe()
        elif self.target_type == 'green':
            self.target_distance = self.vision.get_green_strafe()
        else:
            print(f'Invalid target_type: {self.target_type}')
            pass  # will use the initialized number

        print(f'Attempting to strafe to {self.target_type} located {self.target_distance:.1f}m from starting position at {self.start_pose.Y():.1f}m')

    def execute(self) -> None:  # 50 loops per second. (0.02 seconds per loop)
        # should drive robot a max of ~1 m/s when climbing on fully tilted charge station

        current_time = wpilib.Timer.getFPGATimestamp() - self.strafe_start_time
        max_allowed_velocity = self.calculate_maximum_velocity(current_time) if self.auto else self.min_velocity

        if wpilib.RobotBase.isReal():
            current_strafe = self.start_pose.Y() - self.drive.get_pose().Y()
        else:  # don't have swerve pose working in the sim yet, so fake it
            sim_pose = wpilib.SmartDashboard.getNumberArray('drive_pose', [0, 0, 0])
            current_strafe = self.start_pose.Y() - sim_pose[1]

        pid_output = self.strafe_controller.calculate(current_strafe, setpoint=self.target_distance)

        pid_output = pid_output if abs(pid_output) <= 1 else 1 * math.copysign(1, pid_output)  # clamp at max +/- 1
        pid_output = pid_output if abs(pid_output) > 0.2 else 0.2 * math.copysign(1, pid_output)  # clamp at min +/- 0.11
        target_vel = pid_output * max_allowed_velocity  # meters per second
        # SmartDashboard.putNumber('_s_dist_travelled', self.start_pose.Y() - sim_pose[1])
        # SmartDashboard.putNumber('_s_setpoint', self.target_distance)
        # SmartDashboard.putNumber('_s_pid', pid_output)
        # SmartDashboard.putNumber('_s_sim_y', sim_pose[1])
        # SmartDashboard.putNumber('_s_error', self.strafe_controller.getPositionError())
        # SmartDashboard.putBoolean('_s_atsp', self.strafe_controller.atSetpoint())

        # remember to scale the velocity for the drive function - divide input by max
        self.drive.drive(xSpeed=0, ySpeed=target_vel/dc.kMaxSpeedMetersPerSecond, rot=0, fieldRelative=False, rate_limited=False)
        
    def isFinished(self) -> bool:
        return self.strafe_controller.atSetpoint()  # WTF is this not returning True?

    def end(self, interrupted: bool) -> None:
        # self.container.drive.setX()  # will not stay this way unless we are in autonomous
        self.drive.drive(xSpeed=0, ySpeed=0, rot=0, fieldRelative=False, rate_limited=False)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    # use a logit function to transition from fast initial move to slow one at the end, like this:  ------\_____
    # starts at self.max_velocity and decays to self.min_velocity
    def calculate_maximum_velocity(self, elapsed_time):
        return self.min_velocity + (self.max_velocity - self.min_velocity) * (1 / (1 + math.exp(-self.decay_rate * (self.transition_time_center - elapsed_time))))
