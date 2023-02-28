import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance

import constants


class Vision(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Vision')
        self.counter = 0
        self.ntinst = NetworkTableInstance.getDefault()

        self.pole_targets = 0
        self.pole_distance = 0
        self.pole_rotation = 0

        self.cone_targets = 0
        self.cone_distance = 0
        self.cone_rotation = 0

        self.cube_targets = 0
        self.cube_distance = 0
        self.cube_rotation = 0

        self.ballcam_table = self.ntinst.getTable('BallCam')
        self.camera_dict = {'yellow': {}, 'purple': {}, 'green': {}}
        self.camera_values = {}

        for key in self.camera_dict.keys():
            self.camera_dict[key].update({'targets_entry': self.ballcam_table.getEntry(f"/{key}/targets")})
            self.camera_dict[key].update({'distance_entry': self.ballcam_table.getEntry(f"/{key}/distance")})
            self.camera_dict[key].update({'rotation_entry': self.ballcam_table.getEntry(f"/{key}/rotation")})

            self.camera_values[key] = {}
            self.camera_values[key].update({'targets': 0})
            self.camera_values[key].update({'distance': 0})
            self.camera_values[key].update({'rotation': 0})

    def periodic(self) -> None:
        self.counter += 1

        # update five times a second
        if self.counter % 20 == 0:

            if wpilib.RobotBase.isSimulation():
                SmartDashboard.putNumber('match_time', wpilib.Timer.getFPGATimestamp())
            else:
                SmartDashboard.putNumber('match_time', DriverStation.getMatchTime())

            for key in self.camera_dict.keys():
                self.camera_values[key]['targets'] = self.camera_dict[key]['targets_entry'].getDouble(0)
                self.camera_values[key]['distance_entry'] = self.camera_dict[key]['distance_entry'].getDouble(0)
                self.camera_values[key]['rotation_entry'] = self.camera_dict[key]['rotation_entry'].getDouble(0)

            # update pole values separately
            self.pole_targets = self.camera_dict['green']['targets_entry'].getDouble(0)
            self.pole_distance = self.camera_dict['green']['distance_entry'].getDouble(0)
            self.pole_rotation = self.camera_dict['green']['rotation_entry'].getDouble(0)