import commands2
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from subsystems.swerve import Swerve

import math

class ChargeStationBalance(commands2.CommandBase):

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('ChargeStationBalance')
        self.container = container
        self.addRequirements(container.drive)
        self.roll_controller = PIDController(1/45, 0, 0)
        self.roll_controller.enableContinuousInput(0, 360) 

        self.past_angles = [-(math.copysign(180, -self.container.drive.navx.getRoll()) + self.container.drive.navx.gerRoll())/45]*5
        self.angle_list_index = 0

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.print_start_message()
        # setting initial speed to angle so that when we hold button it doesn't rapidly switch between 0 and proper speed
        # self.container.drive.drive(self.container.drive.navx.getPitch()/45, 0, 0, False,False)

    def execute(self) -> None:  # 50 loops per second. (0.02 seconds per loop)
        # should drive robot a max of ~1 m/s when climbing on fully tilted charge station
        # will probably need adjustment of some kind
        self.angle_list_index %= 5
        self.past_angles[self.angle_list_index] = -(math.copysign(180, -self.container.drive.navx.getRoll()) + self.container.drive.navx.gerRoll())/45
        target_vel = self.roll_controller.calculate(self.container.drive.navx.getRoll() + 180, setpoint=0)
        # target_vel = ((self.container.drive.navx.getRoll() + 180) % 360)/45
        SmartDashboard.putNumber('_target_vel', target_vel)
        if math.fabs(target_vel) > 1/6: target_vel = 1/6*math.copysign(1, target_vel)
        self.container.drive.drive(target_vel, 0, 0, False,False)
        
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.container.drive.setX()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")