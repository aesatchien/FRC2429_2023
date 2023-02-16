#!/usr/bin/env python3

import wpilib
from robotpy_ext.common_drivers.distance_sensors import SharpIR2Y0A41


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.joystick = wpilib.Joystick(0)

        self.entry_sensor = SharpIR2Y0A41(0)
        self.exit_sensor = SharpIR2Y0A41(1)

        self.entry_motor = wpilib.Spark(0)
        self.belt_motor = wpilib.Spark(1)
        self.shooter_motor = wpilib.Spark(2)

    def teleopPeriodic(self) -> None:
        if self.joystick.getRawButton(1):
            self.shooter_motor.set(1)
        else:
            self.shooter_motor.set(0)

        if self.joystick.getRawButton(2):
            self.entry_motor.set(1)
        else:
            self.entry_motor.set(0)

        if self.joystick.getRawButton(11):
            self.belt_motor.set(1)
        elif self.joystick.getRawButton(12):
            self.belt_motor.set(-1)
        else:
            self.belt_motor.set(0)


if __name__ == "__main__":
    wpilib.run(MyRobot)
