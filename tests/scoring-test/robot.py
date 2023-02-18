#!/usr/bin/env python3

"""  20230215 CJH
Test robot designed to:
 drive any of the scoring system motors from a sparkmax/neo combo with a variable output scale
 read from the default encoder
 read from the absolute encoder
 check a limit switch
"""

import wpilib
import rev


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:

        self.joystick = wpilib.Joystick(0)
        self.scale = 0.2
        self.counter = 0

        """ CAN IDs
        k_turret_motor_port = 9 # sparkmax, inverted = True
        k_elevator_motor_port = 10 # sparkmax, inverted = True (up is positive)
        k_wrist_motor_port = 11 # sparkmax
        k_arm_motor_port = 12 # sparkmax
        """

        self.can_id = 10
        self.controller = rev.CANSparkMax(self.can_id, rev.CANSparkMax.MotorType.kBrushless)
        self.controller.setInverted(True)

        self.spark_encoder = self.controller.getEncoder()

        self.spark_encoder_conversion_factor = 0.25  # elevator gear reduction is 16:1 and circumference of 16-tooth is 4.05in
        self.spark_encoder.setPositionConversionFactor(self.spark_encoder_conversion_factor)

        self.limit_switch_port = 0
        self.limit_switch = wpilib.DigitalInput(self.limit_switch_port)

    def teleopPeriodic(self) -> None:
        self.counter += 1
        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)
        b3 = self.joystick.getRawButton(3)

        # drive the mechanism with the stick
        stick = - self.joystick.getY()
        self.controller.set(stick * self.scale)

        if b1:  # go slow on the stick
            self.scale = 0.2
            print(f'Set scale to {self.scale:<30}', end='\r')
        if b2:  # go fast on the stick
            self.scale = 1
            print(f'Set scale to {self.scale:<30}',  end='\r')
        if b3:  # report positions
            msg = f'default encoder: {self.spark_encoder.getPosition():0.1f}  limit switch: {self.limit_switch.get()}'
            print(msg, end='\r')

        if self.counter % 10 == 0:  # report values to dashboard
            wpilib.SmartDashboard.putNumber("canid", self.can_id)
            wpilib.SmartDashboard.putNumber("output", stick)
            wpilib.SmartDashboard.putNumber("encoder", self.spark_encoder.getPosition())
            wpilib.SmartDashboard.putNumber("absolute_encoder", self.absolute_encoder.getPosition())
            wpilib.SmartDashboard.putNumber("limit", self.limit_switch.get())
            #wpilib.SmartDashboard.putNumber("alternate_encoder", self.alternate_encoder.getPosition())`


if __name__ == "__main__":
    wpilib.run(MyRobot)
