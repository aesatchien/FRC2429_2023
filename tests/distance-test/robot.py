#!/usr/bin/env python

"""
2023 0215 CJH
Test for getting range measurements back from the PWF TimeofFlight sensor
10.24.29.2:5812 (172.22.11.2:5812)  - should be a web page that lets you config the TOF sensor, IF IS IT PLUGGED IN

"""

import wpilib
# from robotpy_ext.common_drivers.distance_sensors import SharpIR2Y0A21
from playingwithfusion import TimeOfFlight

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.joystick = wpilib.Joystick(0)
        self.distance_sensor = TimeOfFlight(13)  # ships with an ID of 0
        self.distance_sensor.setRangingMode(TimeOfFlight.RangingMode.kMedium, 50)
        self.distance_sensor.setRangeOfInterest()

        self.led_relay = wpilib.Relay(0,direction=wpilib.Relay.Direction.kForwardOnly)
        self.led_relay.set(wpilib.Relay.Value.kOff)
        self.relay_on = False

        self.counter = 0
        #self.distance_sensor.

    def robotPeriodic(self) -> None:

        self.counter += 1

        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)
        b3 = self.joystick.getRawButton(3)
        b4 = self.joystick.getRawButton(4)

        if b1:
            self.distance_sensor.identifySensor()
        if b2:
            msg = f'distance: {self.distance_sensor.getRange():0.1f}'
            print(msg, end='\r')
        if b3:
            if self.relay_on:
                self.relay_on = False
                self.led_relay.set(wpilib.Relay.Value.kOff)
        if b4:
            if not self.relay_on:
                self.relay_on = True
                self.led_relay.set(wpilib.Relay.Value.kForward)

        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber("COUNTER", self.counter)
            wpilib.SmartDashboard.putNumber("distance", self.distance_sensor.getRange())  # should be mm to target
            wpilib.SmartDashboard.putNumber("status", self.distance_sensor.getStatus())
            wpilib.SmartDashboard.putNumber("status", self.distance_sensor.getRangeSigma())
            wpilib.SmartDashboard.putNumber("serial", self.distance_sensor.getSerialNumber())  # should be mm to target
            wpilib.SmartDashboard.putNumber("relay", self.led_relay.get())


if __name__ == "__main__":
    wpilib.run(MyRobot)
