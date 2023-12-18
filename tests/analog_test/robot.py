#!/usr/bin/env python3

import wpilib

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.analog_0 = wpilib.AnalogInput(0)
        self.digital_0 = wpilib.DigitalInput(0)
        self.digital_1 = wpilib.DigitalInput(1)
        self.digital_2 = wpilib.DigitalInput(2)

    def teleopPeriodic(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        wpilib.SmartDashboard.putNumber("analog 0", self.analog_0.getVoltage())
        wpilib.SmartDashboard.putBoolean("digital 0", self.digital_0.get())
        wpilib.SmartDashboard.putBoolean("digital 1", self.digital_1.get())
        wpilib.SmartDashboard.putBoolean("digital 2", self.digital_2.get())

if __name__ == "__main__":
    wpilib.run(MyRobot)
