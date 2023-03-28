import commands2
from wpilib import SmartDashboard
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from subsystems.swerve import Swerve

class ChargeStationBalance(commands2.CommandBase):

    def __init__(self, container, drive: Swerve, velocity=60, tolerance=4, max_feed_forward=0.5, auto=False) -> None:
        super().__init__()
        self.setName('ChargeStationBalance')
        self.container = container
        self.drive = drive
        self.addRequirements(drive)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.print_start_message()
        # setting initial speed to angle so that when we hold button it doesn't rapidly switch between 0 and proper speed
        self.drive.setModuleStates([SwerveModuleState(self.drive.navx.getPitch()/25, Rotation2d.fromDegrees(0))]*4)

    def execute(self) -> None:  # 50 loops per second. (0.02 seconds per loop)
        # should drive robot a max of ~1 m/s when climbing on fully tilted charge station
        self.drive.setModuleStates([[SwerveModuleState(self.drive.navx.getPitch()/25, Rotation2d.fromDegrees(0))]*4])
        
<<<<<<< HEAD
=======
        roc_angle = (self.pitch-self.prev_pitch) / 0.02  # angle / second; the derivative (roc='rate of change') of the angle
        roc_angle_filtered = self.median_filter.calculate(roc_angle)
        self.data_array[self.count % 5] = roc_angle_filtered

        # not sure what the point of this is...because won't the PID controller speed up the robot in the beginning anyways?
        # we're not using PID, only feed forward, and it's not enough to pull the robot up the slope
        self.count += 1
        if self.count <= 5:
            self.initialDerivSign = math.copysign(1, roc_angle_filtered)

        if (self.count < 100 and self.auto is True): # in the initial second, let us climb faster
            speed_boost = 2
        elif (self.count > 100 and self.count < 150 and self.auto is True):
            speed_boost = 1.5  # in the initial second, let us climb faster
        else:
            speed_boost = 1

        if abs(self.pitch) > self.tolerance:
            # if robot is pitched downwards, drive backwards, or if robot is pitched upwards, drive forwards
            # use the value of the angle as a feedback parameter for feed forward
            sign = math.copysign(1, self.pitch)
            feed_forward = sign * abs(self.pitch) * (self.max_feed_forward/10)  # assume max V for 10 degrees <--why are we assuming this...?
            feed_forward = min(self.max_feed_forward, feed_forward) if sign > 0 else max(-self.max_feed_forward, feed_forward)
            feed_forward *= speed_boost
                
            self.drive.drive_forwards_vel(sign * self.velocity * speed_boost, pidSlot=0,
                                          l_feed_forward=feed_forward, r_feed_forward=feed_forward)

        elif math.copysign(1, roc_angle_filtered) != self.initialDerivSign or roc_angle_filtered <= constants.k_deriv_tolerance:
            self.drive.drive_forwards_vel(0, pidSlot=0)
            # stop condition: if the ROC of the pitch changes signs AND the ROC of the pitch is below some value, then STOP.
            #                AND, the the pitch of the robot is <= some value
            #              

        SmartDashboard.putNumberArray("dpitch", self.data_array)
        SmartDashboard.putNumber("dpitch_filtered: ", roc_angle_filtered)
        SmartDashboard.putNumber("pitch: ", self.pitch)

        if isinstance(self.drive, Drivetrain):
            self.drive.feed()

>>>>>>> a7ea26c64c6ce1fa3cc75e5cf302890517ab98f2
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.drive.setX()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")