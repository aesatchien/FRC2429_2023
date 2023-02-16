#!/usr/bin/env python

"""

"""

import wpilib
# from robotpy_ext.common_drivers.distance_sensors import SharpIR2Y0A21
from playingwithfusion import TimeOfFlight

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.DistSensor = TimeOfFlight(13)
        self.DistSensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)

    def robotPeriodic(self) -> None:
        wpilib.SmartDashboard.putNumber("distance", self.DistSensor.getRange())  # should be mm to target
        wpilib.SmartDashboard.putNumber("status", self.DistSensor.getStatus())  # should be mm to target

if __name__ == "__main__":
    wpilib.run(MyRobot)
