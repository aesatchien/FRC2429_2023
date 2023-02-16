#!/usr/bin/env python3

"""  20230215 CJH
Test robot designed to:
 drive the turret from a sparkmax/neo combo
 read from the default encoder
 read from the absolute encoder
"""

import wpilib
import rev


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.joystick = wpilib.Joystick(0)

        self.turret_controller = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        self.absolute_encoder = self.turret_controller.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.default_encoder = self.turret_controller.getEncoder()

        # not allowed to have an absolute encoder and an alternate encoder
        #self.alternate_encoder = self.turret_controller.getAlternateEncoder(1)

    def teleopPeriodic(self) -> None:
        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)

        stick = self.joystick.getY()
        self.turret_controller.set(stick)

        if b1:
            msg = f'default encoder: {self.default_encoder.getPosition():0.1f}  absolute encoder: {self.absolute_encoder.getPosition():0.1f}'
            print(msg, end='\r')
        if b2:
            pass

        wpilib.SmartDashboard.putNumber("output", stick)
        wpilib.SmartDashboard.putNumber("encoder", self.default_encoder.getPosition())
        wpilib.SmartDashboard.putNumber("absolute_encoder", self.absolute_encoder.getPosition())
        #wpilib.SmartDashboard.putNumber("alternate_encoder", self.alternate_encoder.getPosition())



if __name__ == "__main__":
    wpilib.run(MyRobot)
