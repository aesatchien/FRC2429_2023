#!/usr/bin/env python3

"""  20230215 CJH
Test robot designed to:
 drive the turret from a sparkmax/neo combo
 read from the default encoder
 read from the absolute encoder
 reset the spark encoder based on absolute measurement
 set the offset of the absolute encoder so that stow is 0 degrees
"""

import wpilib
import rev


# noinspection PyAttributeOutsideInit
class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.sample_count = 50
        self.encoder_distance_values = [0] * self.sample_count
        self.encoder_absolute_values = [0] * self.sample_count
        self.counter = 0

        self.absolute_position_offset = 0.842  # this is what the absolute encoder reports when in stow position

        # create objects for robot hardware
        self.joystick = wpilib.Joystick(0)

        # get a can sparkmax for the turret
        self.turret_controller = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        self.turret_controller.setInverted(True)

        self.spark_absolute_encoder = self.turret_controller.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.spark_encoder = self.turret_controller.getEncoder()

        # need to test this out with several angles
        self.spark_encoder_conversion_factor = 360 / 462.2  # Armabot has 462.2:1 gear ratio?  Circle has 360 degrees.
        self.spark_encoder.setPositionConversionFactor(self.spark_encoder_conversion_factor)

        # use the absolute encoder in analog mode - digital documentation is a pain still
        # need the turret encoder to be set to analog (POSSIBLE jumper change)
        self.analog_abs_encoder = wpilib.AnalogEncoder(1)  # plug the analog encoder into channel 1
        self.analog_conversion_factor = 360.0  # It reports 1 rotation when the turret rotates 1  - convert to degrees
        self.analog_abs_encoder.setDistancePerRotation(self.analog_conversion_factor)

        # not allowed to have an absolute encoder and an alternate encoder
        #self.alternate_encoder = self.turret_controller.getAlternateEncoder(1)

    def teleopPeriodic(self) -> None:
        self.counter += 1

        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)
        b3 = self.joystick.getRawButton(3)

        stick = - self.joystick.getY()
        self.turret_controller.set(stick)

        self.encoder_distance_values[self.counter % self.sample_count] = self.analog_abs_encoder.getDistance()
        self.encoder_absolute_values[self.counter % self.sample_count] = self.analog_abs_encoder.getAbsolutePosition()

        if b1:  # report positions (noisy)
            msg = f'default encoder: {self.spark_encoder.getPosition():0.1f}  absolute encoder: {self.analog_abs_encoder.getAbsolutePosition():0.1f}'
            print(msg, end='\r')
        if b2:
            # update the spark's position with the absolute encoder average
            absolute_encoder_avg = sum(self.encoder_distance_values) / self.sample_count
            response = self.spark_encoder.setPosition(absolute_encoder_avg)
            print(f'set sparkmax postion offset to {self.absolute_position_offset} with response {response}')

        if b3:
            # set the position offset of the absolute encoder
            self.absolute_position_offset = sum(self.encoder_absolute_values) / self.sample_count
            self.analog_abs_encoder.setPositionOffset(self.absolute_position_offset)
            print(f'set postion offset to {self.absolute_position_offset}')

        if self.counter % 10 == 0:
            absolute_encoder_avg = sum(self.encoder_distance_values) / self.sample_count
            absolute_encoder_position_avg = sum(self.encoder_absolute_values) / self.sample_count
            wpilib.SmartDashboard.putNumber("output", stick)
            wpilib.SmartDashboard.putNumber("encoder", self.spark_encoder.getPosition())
            wpilib.SmartDashboard.putNumber("absolute_encoder_abs", self.analog_abs_encoder.getAbsolutePosition())
            wpilib.SmartDashboard.putNumber("absolute_encoder_dist_avg", absolute_encoder_avg)
            #wpilib.SmartDashboard.putNumber("alternate_encoder", self.alternate_encoder.getPosition())


if __name__ == "__main__":
    wpilib.run(MyRobot)
