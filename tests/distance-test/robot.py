#!/usr/bin/env python

"""
2023 0215 CJH
Test for getting range measurements back from the PWF TimeofFlight sensor
10.24.29.2:5812 (172.22.11.2:5812)  - should be a web page that lets you config the TOF sensor, IF IS IT PLUGGED IN

from UM2356 user manual, the status messages returned by the sensor:
 Value RangeStatus string Comment
 0 VL53L1_RANGESTATUS_RANGE_VALID Ranging measurement is valid
 1 VL53L1_RANGESTATUS_SIGMA_FAIL Raised if sigma estimator check is above the internal defined threshold
 2 VL53L1_RANGESTATUS_SIGNAL_FAIL Raised if signal value is below the internal defined threshold
 4 VL53L1_RANGESTATUS_OUTOFBOUNDS_ FAIL Raised when phase is out of bounds
 5 VL53L1_RANGESTATUS_HARDWARE_FAIL Raised in case of HW or VCSEL failure
 7 VL53L1_RANGESTATUS_WRAP_TARGET_ FAIL Wrapped target, not matching phases
 8 VL53L1_RANGESTATUS_PROCESSING_ FAIL Internal algorithm underflow or overflow 14
 14 VL53L1_RANGESTATUS_RANGE_INVALID The reported range is invalid
"""

import wpilib
# from robotpy_ext.common_drivers.distance_sensors import SharpIR2Y0A21
from playingwithfusion import TimeOfFlight

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        # set up objects for robot hardware
        self.joystick = wpilib.Joystick(0)
        self.distance_sensor = TimeOfFlight(13)  # ships with an ID of 0
        self.distance_sensor.setRangingMode(TimeOfFlight.RangingMode.kMedium, 50)
        # self.distance_sensor.setRangeOfInterest()

        self.led_relay = wpilib.Relay(0,direction=wpilib.Relay.Direction.kForwardOnly)
        self.led_relay.set(wpilib.Relay.Value.kOff)
        self.relay_on = False

        self.counter = 0


    def robotPeriodic(self) -> None:

        self.counter += 1

        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)
        b3 = self.joystick.getRawButton(3)
        b4 = self.joystick.getRawButton(4)

        if b1:  # make the sensor blink
            self.distance_sensor.identifySensor()
        if b2:  # print the distance
            msg = f'distance: {self.distance_sensor.getRange():0.1f}'
            print(msg, end='\r')
        if b3:  # toggle relay off
            if self.relay_on:
                self.relay_on = False
                self.led_relay.set(wpilib.Relay.Value.kOff)
        if b4:  # toggle relay on
            if not self.relay_on:
                self.relay_on = True
                self.led_relay.set(wpilib.Relay.Value.kForward)

        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber("counter", self.counter)
            wpilib.SmartDashboard.putNumber("range", self.distance_sensor.getRange())  # should be mm to target
            wpilib.SmartDashboard.putNumber("status", self.distance_sensor.getStatus())  # 0 means range is valid
            wpilib.SmartDashboard.putNumber("sigma", self.distance_sensor.getRangeSigma())
            wpilib.SmartDashboard.putNumber("serial", self.distance_sensor.getSerialNumber())
            wpilib.SmartDashboard.putNumber("relay", self.led_relay.get())


if __name__ == "__main__":
    wpilib.run(MyRobot)
