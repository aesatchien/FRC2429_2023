#!/usr/bin/env python

"""
2023 0215 CJH
Test for getting rage measurements back from the PWF TimeofFlight sensor
10.24.29.2:5812  - should be a web page that lets you config the TOF sensor
"""

import wpilib
# from robotpy_ext.common_drivers.distance_sensors import SharpIR2Y0A21
from playingwithfusion import TimeOfFlight

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.joystick = wpilib.Joystick(0)
        self.DistSensor = TimeOfFlight(0)  # ships with an ID of 0
        self.DistSensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)
        self.counter = 0
        # self.DistSensor.

    def robotPeriodic(self) -> None:

        self.counter += 1

        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)
        if b1:
            self.DistSensor.identifySensor()
        if b2:
            msg = f'distance: {self.DistSensor.getRange():0.1f}'
            print(msg, end='\r')

        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber("distance", self.DistSensor.getRange())  # should be mm to target
            wpilib.SmartDashboard.putNumber("status", self.DistSensor.getStatus())  # should be mm to target

if __name__ == "__main__":
    wpilib.run(MyRobot)
