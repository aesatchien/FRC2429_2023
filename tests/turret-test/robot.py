#!/usr/bin/env python3

import wpilib
import rev


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.joystick = wpilib.Joystick(0)

        self.turret_controller = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        #self.absolute_encoder =  self.turret_controller.getAbsoluteEncoder(encoderType= rev.SparkMaxAbsoluteEncoder)
        self.default_encoder = self.turret_controller.getEncoder()
        self.alternate_encoder = self.turret_controller.getAlternateEncoder(1)

    def teleopPeriodic(self) -> None:
        b1 = self.joystick.getRawButton(0)
        b2 = self.joystick.getRawButton(1)

        stick = self.joystick.getY()
        self.turret_controller.set(stick)


        if b1:
            pass
        if b2:
            pass


        wpilib.SmartDashboard.putNumber("output", stick)
        wpilib.SmartDashboard.putNumber("encoder", self.default_encoder.getPosition())
        #wpilib.SmartDashboard.putNumber("absolute_encoder", self.absolute_encoder.getPosition())
        wpilib.SmartDashboard.putNumber("alternate_encoder", self.alternate_encoder.getPosition())


if __name__ == "__main__":
    wpilib.run(MyRobot)
