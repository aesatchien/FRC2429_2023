#!/usr/bin/env python3

import rev
import wpilib
from wpimath.filter import MedianFilter, LinearFilter
import random


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.joystick = wpilib.Joystick(0)
        self.printing = False
        self.print_button = False

        self.turning_motors = {'lf': {'id': 20}, 'rf': {'id': 24}, 'lb': {'id': 22},'rb': { 'id': 26}, }

        for (key, motor_info) in self.turning_motors.items():
            motor = rev.CANSparkMax(motor_info['id'], rev.CANSparkMax.MotorType.kBrushless)
            temp_dict = {
                'id': motor_info['id'],
                'motor': motor,
                'encoder': motor.getEncoder(),
                'analog_encoder': motor.getAnalog(),
                'filter': MedianFilter(50)
            }
            self.turning_motors[key].update(temp_dict)

        # print(self.turning_motors)

    def teleopPeriodic(self):

        b1 = self.joystick.getRawButton(1)
        if b1 and not self.print_button:  # don't print twice in a row
            self.printing = True
            print(f'\nFiltered abs encoders taken at {wpilib.Timer.getFPGATimestamp():.1f}s')  # print a blank line
        else:
            self.printing = False

        for idx, (key, motor_info) in enumerate(self.turning_motors.items()):
            if wpilib.RobotBase.isReal():
                analog_position = motor_info['analog_encoder'].getPosition()
            else:  # simulate some noise
                analog_position = random.gauss(float(motor_info['id']), .25)
            filtered_analog_position = motor_info['filter'].calculate(analog_position)  # smooth out the noise
            wpilib.SmartDashboard.putNumber(f"{key}_abs_encoder_{motor_info['id']}", analog_position)
            wpilib.SmartDashboard.putNumber(f"{key}_filtered_abs_{motor_info['id']}", filtered_analog_position)
            wpilib.SmartDashboard.putNumber(f"{key}_reg_encoder_{motor_info['id']}", motor_info['encoder'].getPosition())

            if self.printing:
                print(f"{key}_filtered_abs_{motor_info['id']} : {filtered_analog_position: .3f}")

        self.print_button = b1  # remember what it was

if __name__ == "__main__":
    wpilib.run(Robot)