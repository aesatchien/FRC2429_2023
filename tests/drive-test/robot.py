#!/usr/bin/env python3

import wpilib
import wpilib.drive
import rev
import ctre

from robotpy_ext.common_drivers.distance_sensors import SharpIR2Y0A21, SharpIR2Y0A41


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.joystick = wpilib.Joystick(0)

        self.drive_l1 = ctre.WPI_VictorSPX(1)  #
        self.drive_l2 = ctre.WPI_VictorSPX(2)  #
        self.drive_r1 = ctre.WPI_VictorSPX(3)  #
        self.drive_r2 = ctre.WPI_VictorSPX(4)
        self.encoder_l = wpilib.Encoder(0, 1)
        self.encoder_r = wpilib.Encoder(2, 3)

        self.drive_l1.setInverted(True)
        self.drive_l2.setInverted(True)

        self.drive_l2.follow(self.drive_l1)
        self.drive_r2.follow(self.drive_r1)

        self.drive = wpilib.drive.DifferentialDrive(self.drive_l1, self.drive_r1)

    def teleopPeriodic(self) -> None:

        self.drive.arcadeDrive(self.joystick.getY(), -self.joystick.getX(), False)

        wpilib.SmartDashboard.putNumber("Encoder L", self.encoder_l.get())
        wpilib.SmartDashboard.putNumber("Encoder R", self.encoder_r.get())


if __name__ == "__main__":
    wpilib.run(MyRobot)
