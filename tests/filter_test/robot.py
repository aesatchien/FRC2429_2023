#!/usr/bin/env python

"""
2023 0327 CJH
Test for getting filtered data from an analog sensor
"""

import wpilib
# from robotpy_ext.common_drivers.distance_sensors import SharpIR2Y0A21
from wpilib import AnalogInput
from wpimath.estimator import KalmanFilter_1_1_1, ExtendedKalmanFilter_1_1_1
import random, math

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        # set up objects for robot hardware
        self.joystick = wpilib.Joystick(0)
        self.analog = AnalogInput(0)
        #self.kalman = KalmanFilter_1_1_1()


        self.counter = 0


    def robotPeriodic(self) -> None:

        self.counter += 1

        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)

        if b1:  # make the sensor blink
            pass
        if b2:  # print the distance
            pass
            msg = f'distance: {0:0.1f}'
            print(msg, end='\r')


        if self.counter % 2 == 0:
            wpilib.SmartDashboard.putNumber("counter", self.counter)

            wpilib.SmartDashboard.putNumber("range", math.sin(3.14*self.counter/100) + random.gauss(0,0.1))


if __name__ == "__main__":
    wpilib.run(MyRobot)
