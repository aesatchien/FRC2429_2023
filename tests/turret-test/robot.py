#!/usr/bin/env python3

"""  20230215 CJH
Test robot designed to:
 drive the turret from a sparkmax/neo combo
 read from the default encoder
 read from the absolute encoder
"""

import wpilib
import rev


# noinspection PyAttributeOutsideInit
class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.encoder_values = [0] * 50
        self.counter = 0

        self.joystick = wpilib.Joystick(0)

        self.turret_controller = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        self.turret_controller.setInverted(True)

        self.digital_absolute_encoder = self.turret_controller.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.default_encoder = self.turret_controller.getEncoder()

        # need to test this out with several angles
        self.default_encoder_conversion_factor = 360/462.0  # Armabot has 442:1 gear ratio?  Circle has 360 degrees.
        self.default_encoder.setPositionConversionFactor(self.default_encoder_conversion_factor)

        # same here, and need the turret encoder to be set to analog (jumper change)
        self.analog_absolute_encoder = wpilib.AnalogEncoder(1)  # plug the analog encoder into channel 1
        # self.analog_conversion_factor = 5/360.0  # 5V is 360 degrees
        self.analog_conversion_factor = 360
        self.analog_absolute_encoder.setDistancePerRotation(self.analog_conversion_factor)

        # not allowed to have an absolute encoder and an alternate encoder
        #self.alternate_encoder = self.turret_controller.getAlternateEncoder(1)

    def teleopPeriodic(self) -> None:
        self.counter += 1

        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)

        stick = - self.joystick.getY()
        self.turret_controller.set(stick)

        if b1:
            msg = f'default encoder: {self.default_encoder.getPosition():0.1f}  absolute encoder: {self.analog_absolute_encoder.getAbsolutePosition():0.1f}'
            print(msg, end='\r')
        if b2:
            # update the spark's position with the absolute encoder
            current_angle = self.analog_absolute_encoder.getDistance()
            self.default_encoder.setPosition(current_angle)

        self.encoder_values[self.counter % len(self.encoder_values)] = self.analog_absolute_encoder.getDistance()

        if self.counter % 10 == 0:
            absolute_encoder_avg = sum(self.encoder_values) / len(self.encoder_values)

            wpilib.SmartDashboard.putNumber("output", stick)
            wpilib.SmartDashboard.putNumber("encoder", self.default_encoder.getPosition())
            wpilib.SmartDashboard.putNumber("absolute_encoder_abs", self.analog_absolute_encoder.getAbsolutePosition())
            wpilib.SmartDashboard.putNumber("absolute_encoder_dist", absolute_encoder_avg)
            #wpilib.SmartDashboard.putNumber("alternate_encoder", self.alternate_encoder.getPosition())



if __name__ == "__main__":
    wpilib.run(MyRobot)
