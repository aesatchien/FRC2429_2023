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
        k_turret_motor_port = 9 # sparkmax
        k_elevator_motor_port = 10 # sparkmax
        k_wrist_motor_port = 11 # sparkmax
        k_arm_motor_port = 12 # sparkmax
        """

        self.can_id = 10
        self.controller = rev.CANSparkMax(self.can_id, rev.CANSparkMax.MotorType.kBrushless)
        self.absolute_encoder = self.controller.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.default_encoder = self.controller.getEncoder()

        self.limit_switch_port = 0
        self.limit_switch = wpilib.DigitalInput(self.limit_switch_port)

        # not allowed to have an absolute encoder and an alternate encoder
        #self.alternate_encoder = self.turret_controller.getAlternateEncoder(1)

    def teleopPeriodic(self) -> None:
        self.counter += 1
        b1 = self.joystick.getRawButton(1)
        b2 = self.joystick.getRawButton(2)
        b3 = self.joystick.getRawButton(3)

        stick = - self.joystick.getY()

        if b3:
            msg = f'default encoder: {self.default_encoder.getPosition():0.1f}  absolute encoder: {self.absolute_encoder.getPosition():0.1f}'
            print(msg, end='\r')
        if b2:
            self.scale = 1
            print(f'Set scale to {self.scale:<30}',  end='\r')
        if b1:
            self.scale = 0.2
            print(f'Set scale to {self.scale:<30}',  end='\r')

        self.controller.set(stick * self.scale)

        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber("canid", self.can_id)
            wpilib.SmartDashboard.putNumber("output", stick)
            wpilib.SmartDashboard.putNumber("encoder", self.default_encoder.getPosition())
            wpilib.SmartDashboard.putNumber("absolute_encoder", self.absolute_encoder.getPosition())
            wpilib.SmartDashboard.putNumber("limit", self.limit_switch.get())
            #wpilib.SmartDashboard.putNumber("alternate_encoder", self.alternate_encoder.getPosition())`



if __name__ == "__main__":
    wpilib.run(MyRobot)
