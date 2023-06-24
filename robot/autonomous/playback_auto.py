import commands2
import commands2.cmd as cmd
import json
import math
from wpilib import SmartDashboard
from subsystems.swerve_constants import DriveConstants
from commands.manipulator_toggle import ManipulatorToggle
import constants

from subsystems.led import Led

from commands.arm_move import ArmMove
from commands.turret_move import TurretMove
from commands.turret_toggle import TurretToggle
from commands.elevator_move import ElevatorMove
from commands.wrist_move import WristMove
from commands.manipulator_toggle import ManipulatorToggle
from commands.compressor_toggle import CompressorToggle
from commands.generic_drive import GenericDrive  # Siraaj's way of moving all subsystems
from commands.toggle_high_pickup import ToggleHighPickup
from commands.gyro_reset import GyroReset
from commands.co_stow import CoStow
from commands.manipulator_auto_grab import ManipulatorAutoGrab
from commands.turret_reset import TurretReset

from autonomous.auto_rotate_swerve import AutoRotateSwerve
from autonomous.auto_strafe_swerve import AutoStrafeSwerve
from autonomous.auto_setup_score import AutoSetupScore

class PlaybackAuto(commands2.CommandBase):
    # look, ma, no path planning!
    # this is basically another robotcontainer, but I don't think there's a better way to do this.

    def __init__(self, container, input_log_path) -> None:
        super().__init__()

        self.setName('Playback Auto')
        self.container = container
        self.input_log_path = input_log_path
        # Take over all subsystems
        self.addRequirements(self.container.drive)
        self.addRequirements(self.container.turret)
        self.addRequirements(self.container.arm)
        self.addRequirements(self.container.wrist)
        self.addRequirements(self.container.elevator)
        self.addRequirements(self.container.pneumatics)
        self.addRequirements(self.container.vision)
        self.addRequirements(self.container.led)


        self.subsystem_list = ['turret', 'elevator', 'wrist', 'arm']

        self.command_dict = {
            'UP': {
                'turret': TurretMove(self.container, self.container.turret, direction="up", wait_to_finish=False),
                'elevator': ElevatorMove(self.container, self.container.elevator, direction="up", wait_to_finish=False, drive_controls=True),
                'arm': ArmMove(self.container, self.container.arm, direction="up", wait_to_finish=False),
                'wrist': WristMove(self.container, self.container.wrist, direction="down", wait_to_finish=False),
            },
            'DOWN': {
                'turret': TurretMove(self.container, self.container.turret, direction="down", wait_to_finish=False),
                'elevator': ElevatorMove(self.container, self.container.elevator, direction="down", wait_to_finish=False, drive_controls=True),
                'arm': ArmMove(self.container, self.container.arm, direction="down", wait_to_finish=False),
                'wrist': WristMove(self.container, self.container.wrist, direction="up", wait_to_finish=False),
            },
            'UP_DRIVE': {
                'turret': GenericDrive(self.container, self.container.turret, max_velocity=constants.k_PID_dict_vel_turret["SM_MaxVel"], input_type='dpad', direction=1),
                'elevator': GenericDrive(self.container, self.container.elevator, max_velocity=constants.k_PID_dict_vel_elevator["SM_MaxVel"], input_type='dpad', direction=1),
                'arm': GenericDrive(self.container, self.container.arm, max_velocity=constants.k_PID_dict_vel_arm["SM_MaxVel"], input_type='dpad', direction=1),
                'wrist': GenericDrive(self.container, self.container.wrist, max_velocity=constants.k_PID_dict_vel_wrist["SM_MaxVel"], control_type='velocity', input_type='dpad', direction=1, invert_axis=True),
            },
            'DOWN_DRIVE': {
                'turret': GenericDrive(self, self.container.turret, max_velocity=constants.k_PID_dict_vel_turret["SM_MaxVel"], input_type='dpad', direction=-1),
                'elevator': GenericDrive(self, self.container.elevator, max_velocity=constants.k_PID_dict_vel_elevator["SM_MaxVel"], input_type='dpad', direction=-1),
                'arm': GenericDrive(self, self.container.arm, max_velocity=constants.k_PID_dict_vel_arm["SM_MaxVel"], input_type='dpad', direction=-1),
                'wrist': GenericDrive(self, self.container.wrist, max_velocity=constants.k_PID_dict_vel_wrist["SM_MaxVel"], control_type='velocity', input_type='dpad', direction=-1, invert_axis=True),
            },
            'NONE': {
                'none': cmd.nothing(),
            }
        }

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        with open(self.input_log_path, 'r') as input_json:
            self.input_log = json.load(input_json)

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        self.line_count = 1

    def execute(self) -> None:

        # --------------- SWERVE ---------------

        current_inputs = self.input_log[self.line_count]
        previous_inputs = self.input_log[self.line_count-1]

        if current_inputs['driver_controller']['button']['LB']:
            slowmode_multiplier = constants.k_slowmode_multiplier
        elif current_inputs['driver_controller']['axis']['axis2']:
            slowmode_multiplier = 1.5 * constants.k_slowmode_multiplier
        else: 
            slowmode_multiplier = 1

        self.container.drive.drive(-self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis1']),
                                   self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis0']),
                                   -self.input_transform(slowmode_multiplier*current_inputs['driver_controller']['axis']['axis4']),
                                   fieldRelative=True, rate_limited=False, keep_angle=True)
        

        # --------------- DRIVER CONTROLLER ---------------

        if current_inputs['driver_controller']['button']['A'] and not previous_inputs['driver_controller']['button']['A']:
            commands2.CommandScheduler.getInstance().schedule(AutoSetupScore(container=self.container))

        if current_inputs['driver_controller']['button']['B'] and not previous_inputs['driver_controller']['button']['B']:
            commands2.CommandScheduler.getInstance().schedule(GyroReset(container=self.container, swerve=self.container.swerve))

        if current_inputs['driver_controller']['button']['X'] and not previous_inputs['driver_controller']['button']['X']:
            commands2.CommandScheduler.getInstance().schedule(AutoStrafeSwerve(container=self.container, drive=self.container.drive, vision=self.container.vision,
                                                           target_type='tag', auto=True).withTimeout(5))

        if current_inputs['driver_controller']['button']['Y'] and not previous_inputs['driver_controller']['button']['Y']:
            commands2.CommandScheduler.getInstance().schedule(AutoRotateSwerve(container=self, drive=self.container.drive,).withTimeout(2))

        if current_inputs['driver_controller']['button']['RB'] and not previous_inputs['driver_controller']['button']['RB']:
            commands2.CommandScheduler.getInstance().schedule(cmd.runOnce(action=lambda: self.container.wrist.set_driver_flag(state=True)).andThen(
                                                            ReleaseAndStow(container=self).withTimeout(4)).andThen(
                                                            cmd.runOnce(action=lambda: self.container.wrist.set_driver_flag(state=False))))
            
        if current_inputs['driver_controller']['button']['Back'] and not previous_inputs['driver_controller']['button']['Back']:
            commands2.CommandScheduler.getInstance().schedule(CompressorToggle(container=self.container, pneumatics=self.container.pneumatics,
                                                                               force='stop'))

        if current_inputs['driver_controller']['button']['Start'] and not previous_inputs['driver_controller']['button']['Start']:
            commands2.CommandScheduler.getInstance().schedule(CompressorToggle(container=self.container, pneumatics=self.container.pneumatics,
                                                                               force='start'))

        if current_inputs['driver_controller']['button']['POV'] == 0 and not previous_inputs['driver_controller']['button']['POV'] == 0:
            commands2.CommandScheduler.getInstance().schedule(self.led.set_indicator_with_timeout(Led.Indicator.RAINBOW, 5))

        if current_inputs['driver_controller']['button']['POV'] == 180 and not previous_inputs['driver_controller']['button']['POV'] == 180:
                    commands2.CommandScheduler.getInstance().schedule(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics))

        if current_inputs['driver_controller']['button']['POV'] == 270 and not previous_inputs['driver_controller']['button']['POV'] == 270:
                    commands2.CommandScheduler.getInstance().schedule(self.led.set_indicator_with_timeout(Led.Indicator.RSL, 5))


        # --------------- OPERATOR CONTROLLER ---------------


        subsystem_keys = [self.subsystem_list[i] for i, key in enumerate(['A', 'B', 'X', 'Y']) if current_inputs['co_driver_controller']['button'][key]]

        if current_inputs['co_driver_controller']['button']['POV'] == 0:
            for subsystem in subsystem_keys: self.command_dict['UP_DRIVE'][subsystem].execute() # this is very sketchy

        if current_inputs['co_driver_controller']['button']['POV'] == 90 and not previous_inputs['co_driver_controller']['button']['POV'] == 90:
            for subsystem in subsystem_keys: commands2.CommandScheduler.getInstance().schedule(self.command_dict['UP'][subsystem]) 

        if current_inputs['co_driver_controller']['button']['POV'] == 180:
            for subsystem in subsystem_keys: self.command_dict['DOWN_DRIVE'][subsystem].execute()

        if current_inputs['co_driver_controller']['button']['POV'] == 270 and not previous_inputs['co_driver_controller']['button']['POV'] == 270:
            for subsystem in subsystem_keys: commands2.CommandScheduler.getInstance().schedule(self.command_dict['DOWN'][subsystem]) 

        if current_inputs['co_driver_controller']['button']['LB'] and not previous_inputs['co_driver_controller']['button']['LB']:
            commands2.CommandScheduler.getInstance.schedule(ToggleHighPickup(container=self.container, turret=self.container.turret, elevator=self.container.elevator,
                                                                              wrist=self.container.wrist, pneumatics=self.container.pneumatics, vision=self.container.vision))

        if current_inputs['co_driver_controller']['button']['RB'] and not previous_inputs['co_driver_controller']['button']['RB']:
            ManipulatorAutoGrab(container=self.container, pneumatics=self.container.pneumatics).execute() # again very sketchy.

        if current_inputs['co_driver_controller']['button']['Back'] and not previous_inputs['co_driver_controller']['button']['Back']:
            commands2.CommandScheduler.getInstance.schedule(CoStow(container=self))

        if current_inputs['co_driver_controller']['button']['Start'] and not previous_inputs['co_driver_controller']['button']['Start']:
            commands2.CommandScheduler.getInstance.schedule(TurretReset(container=self, turret=self.turret))

        if current_inputs['co_driver_controller']['button']['LS'] and not previous_inputs['co_driver_controller']['button']['LS']:
            commands2.CommandScheduler.getInstance.schedule(TurretToggle(container=self, turret=self.turret, wait_to_finish=False))

        if current_inputs['co_driver_controller']['axis']['axis2'] > 0.2 and not previous_inputs['co_driver_controller']['axis']['axis2'] > 0.2:
            commands2.CommandScheduler.getInstance.schedule(TurretToggle(container=self, turret=self.turret, wait_to_finish=False))

        if current_inputs['co_driver_controller']['axis']['axis3'] > 0.2 and not previous_inputs['co_driver_controller']['axis']['axis3'] > 0.2:
            commands2.CommandScheduler.getInstance.schedule(TurretToggle(container=self, turret=self.turret, wait_to_finish=False))

        self.line_count += 1

    # stuff from drive_by_joystick_swerve- might it be better and DRYer to have it in the subsystem?
    def apply_deadband(self, value, db_low=DriveConstants.k_inner_deadband, db_high=DriveConstants.k_outer_deadband):
        if abs(value) < db_low:
            return 0
        elif abs(value) > db_high:
            return 1 * math.copysign(1, value)
        else:
            return value

    def input_transform(self, value, a=0.9, b=0.1):
        db_value = self.apply_deadband(value)
        return a * db_value**3 + b * db_value

    def isFinished(self) -> bool:
        return self.line_count >= len(self.input_log)

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
