#!/usr/bin/env python

"""
2023 0320 CJH
Test for seeing if NT will consistently respond to the .running = True from the dash
"""

import wpilib
import commands2


class DashCommand(commands2.CommandBase):

    def __init__(self) -> None:
        super().__init__()
        self.setName('DashCommand')
        self.counter = 0

    def initialize(self) -> None:
        self.counter += 1
        print(f'DashCommand called for the {self.counter}th time at {wpilib.Timer.getFPGATimestamp()}')

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return True

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:

        # set up objects for robot hardware
        self.joystick = wpilib.Joystick(0)
        self.counter = 0

        wpilib.SmartDashboard.putData('DashCommand', DashCommand())

    def robotPeriodic(self) -> None:

        b1 = self.joystick.getRawButton(1)

        if b1:  # send out a message
            pass

if __name__ == "__main__":
    wpilib.run(MyRobot)

