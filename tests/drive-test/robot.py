#!/usr/bin/env python3

import wpilib
import wpilib.drive
import rev


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.joystick = wpilib.Joystick(0)
        self.counter = 0

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.drive_l1 = rev.CANSparkMax(1, motor_type)  #
        self.drive_l2 = rev.CANSparkMax(2, motor_type)  #
        self.drive_r1 = rev.CANSparkMax(3, motor_type)  #
        self.drive_r2 = rev.CANSparkMax(4, motor_type)

        self.encoder_l = self.drive_l1.getEncoder()  # c.f. wpilib.Encoder(2, 3)
        self.encoder_r = self.drive_r1.getEncoder()

        self.drive_l1.setInverted(True)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_l2.setInverted(True)

        self.drive_l2.follow(self.drive_l1)
        self.drive_r2.follow(self.drive_r1)

        self.drive = wpilib.drive.DifferentialDrive(self.drive_l1, self.drive_r1)

    def teleopPeriodic(self) -> None:

        self.drive.arcadeDrive(self.joystick.getY(), -self.joystick.getX(), False)

        self.counter += 1
        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber("Encoder L", self.encoder_l.get())
            wpilib.SmartDashboard.putNumber("Encoder R", self.encoder_r.get())


if __name__ == "__main__":
    wpilib.run(MyRobot)
